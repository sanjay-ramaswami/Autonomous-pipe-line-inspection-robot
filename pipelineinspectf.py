"""
pipelineinspectf.py  —  v2.2
==============================
Pipeline Inspection Robot — Upgraded Control Program
Target   : Raspberry Pi Zero 2 W
Requires : opencv-python-headless, tflite_runtime, pyserial,
           matplotlib, reportlab, flask

Changes in v2.2
---------------
* DATA telemetry parser added to serial_listener():
    DATA,timestamp,x,y,angle,gyro,speed,distance,crack_flag
  — Updates current_distance, robot_status, and new robot_telemetry dict.
  — crack_flag=1 in the DATA packet is honoured as a CRACK event.
  — Malformed packets are logged as warnings; they never crash the program.
* New module-level shared state: robot_telemetry (dict) + telemetry_lock.
* Legacy DIST: message handler preserved for backwards compatibility.

New in v2.0
-----------
* Real-time Flask web dashboard (Thread 3) — http://<pi_ip>:5000
* Live pipeline map embedded in dashboard (base64 PNG, rebuilt on demand)
* Robot status tracking (MOVING / REVERSING / STOPPED)
* Triangle robot-position marker on pipeline map
* All original v1 functionality preserved unchanged

Changes in v2.1
---------------
* SERIAL_PORT updated to /dev/ttyS0
* FILE_MODEL  updated to crack_model.tflite  (path kept as-is)
* FILE_MAP    updated to reports/2D_Map.png
* FILE_REPORT updated to reports/Inspection_Report.pdf
* LOG_FILE    (FILE_CSV) updated to crack_log.csv  (logs/ folder retained)
* FRAME_SKIP  updated to 3  (INFERENCE_INTERVAL from config)
* CRACK_THRESHOLD updated to 0.85
"""

# ---------------------------------------------------------------------------
# Standard-library
# ---------------------------------------------------------------------------
import base64
import csv
import io
import logging
import os
import queue
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Third-party
# ---------------------------------------------------------------------------
import cv2
import serial
from flask import Flask, jsonify, render_template_string, send_file

try:
    from tflite_runtime.interpreter import Interpreter as TFLiteInterpreter
except ImportError:
    from tensorflow.lite.python.interpreter import Interpreter as TFLiteInterpreter

# ---------------------------------------------------------------------------
# Directory / file paths
# ---------------------------------------------------------------------------
DIR_CRACK_IMAGES = Path("crack_images")    # CRACK_FOLDER from config
DIR_LOGS         = Path("logs")
DIR_REPORTS      = Path("reports")

FILE_MODEL  = Path("crack_model.tflite")
FILE_LOG    = DIR_LOGS / "inspection.log"
FILE_CSV    = DIR_LOGS / "crack_log.csv"          # LOG_FILE from config
FILE_MAP    = DIR_REPORTS / "2D_Map.png"           # MAP_IMAGE from config
FILE_REPORT = DIR_REPORTS / "Inspection_Report.pdf"  # REPORT_FILE from config

# ---------------------------------------------------------------------------
# Hardware / tuning constants
# ---------------------------------------------------------------------------
SERIAL_PORT    = "/dev/ttyS0"    # ttyS0 hardware UART on RPi Zero 2 W
SERIAL_BAUD    = 115200
SERIAL_TIMEOUT = 1.0

CAM_INDEX  = 0
CAM_WIDTH  = 320
CAM_HEIGHT = 240
CAM_FPS    = 10
FRAME_SKIP = 3          # INFERENCE_INTERVAL — run AI on 1-of-N frames

CRACK_THRESHOLD = 0.85  # confidence threshold (0.0–1.0)
HB_INTERVAL     = 5.0   # heartbeat to ESP32 in seconds

DASHBOARD_PORT  = 5000
MAP_REBUILD_INTERVAL = 3.0   # rebuild map image every N seconds for dashboard

# ---------------------------------------------------------------------------
# Shared state  (all writes must hold the appropriate lock)
# ---------------------------------------------------------------------------
current_distance = 0.0
distance_lock    = threading.Lock()

robot_status   = "STOPPED"          # MOVING | REVERSING | STOPPED
status_lock    = threading.Lock()

event_log: list[dict] = []
event_log_lock = threading.Lock()

crack_count = 0
block_count = 0
counter_lock = threading.Lock()

# Latest crack image as JPEG bytes (for dashboard)
latest_crack_jpg: bytes | None = None
latest_crack_lock = threading.Lock()

# Pre-rendered map PNG as bytes (rebuilt periodically for the dashboard)
map_png_bytes: bytes | None = None
map_png_lock  = threading.Lock()

# ── v2.2: full telemetry dict populated by DATA messages ──────────────────
robot_telemetry: dict = {
    "timestamp":  0,
    "x":          0.0,
    "y":          0.0,
    "angle":      0.0,
    "gyro":       0.0,
    "speed":      0,
    "distance":   0,
    "crack_flag": 0,
}
telemetry_lock = threading.Lock()
# ──────────────────────────────────────────────────────────────────────────

inspection_done = threading.Event()

ser: serial.Serial | None = None  # opened in main()

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def setup_logging() -> None:
    DIR_LOGS.mkdir(parents=True, exist_ok=True)
    fmt = "%(asctime)s [%(levelname)s] %(threadName)s — %(message)s"
    logging.basicConfig(
        level=logging.DEBUG,
        format=fmt,
        handlers=[
            logging.FileHandler(FILE_LOG),
            logging.StreamHandler(sys.stdout),
        ],
    )

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Directory bootstrap
# ---------------------------------------------------------------------------

def initialise_directories() -> None:
    for d in (DIR_CRACK_IMAGES, DIR_LOGS, DIR_REPORTS):
        d.mkdir(parents=True, exist_ok=True)
        logger.debug("Directory ready: %s", d)


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def initialise_csv() -> None:
    if not FILE_CSV.exists():
        with open(FILE_CSV, "w", newline="") as f:
            csv.writer(f).writerow(["distance", "event", "image", "timestamp"])


def log_event(event: str, image_path: str = "") -> None:
    """Thread-safe: append event to in-memory list and CSV file."""
    global crack_count, block_count

    with distance_lock:
        dist = current_distance

    ts = datetime.now().isoformat(timespec="seconds")
    entry = {"distance": dist, "event": event,
             "image": image_path, "timestamp": ts}

    with event_log_lock:
        event_log.append(entry)

    with open(FILE_CSV, "a", newline="") as f:
        csv.writer(f).writerow([dist, event, image_path, ts])

    with counter_lock:
        if event == "CRACK":
            crack_count += 1
        elif event == "BLOCK":
            block_count += 1

    logger.info("Event — %s @ %.2f m  img=%s", event, dist, image_path or "—")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def initialize_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAM_INDEX}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    logger.info(
        "Camera opened %dx%d @ %d FPS",
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        int(cap.get(cv2.CAP_PROP_FPS)),
    )
    return cap


# ---------------------------------------------------------------------------
# TFLite model
# ---------------------------------------------------------------------------

def load_model() -> tuple:
    if not FILE_MODEL.exists():
        raise FileNotFoundError(
            f"TFLite model not found: {FILE_MODEL}. "
            "Place crack_model.tflite in the working directory."
        )
    interp = TFLiteInterpreter(model_path=str(FILE_MODEL))
    interp.allocate_tensors()
    in_d  = interp.get_input_details()
    out_d = interp.get_output_details()
    _, h, w, _ = in_d[0]["shape"]
    logger.info("TFLite model loaded — input %dx%d", h, w)
    return interp, (h, w), out_d[0]["index"]


# ---------------------------------------------------------------------------
# Crack inference
# ---------------------------------------------------------------------------

def detect_cracks(frame, interpreter, input_size, output_index) -> float:
    import numpy as np  # deferred — saves memory at startup

    h, w   = input_size
    resized = cv2.resize(frame, (w, h))
    rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    in_det = interpreter.get_input_details()[0]
    if in_det["dtype"] == np.uint8:
        blob = rgb.astype(np.uint8)
    else:
        blob = (rgb / 255.0).astype(np.float32)

    interpreter.set_tensor(in_det["index"], np.expand_dims(blob, 0))
    interpreter.invoke()

    out = interpreter.get_tensor(output_index)
    return float(out[0][1]) if out.shape[-1] >= 2 else float(out.flat[0])


# ---------------------------------------------------------------------------
# Pipeline map  (called both at inspection end AND by dashboard refresh)
# ---------------------------------------------------------------------------

def _build_map_figure(save_file: bool = True):
    """
    Build and return a Matplotlib figure of the pipeline.
    If save_file=True, also write it to FILE_MAP on disk.
    Always returns raw PNG bytes for the dashboard.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    with distance_lock:
        total_dist = current_distance
    with event_log_lock:
        snap = list(event_log)

    fig, ax = plt.subplots(figsize=(12, 3))
    fig.patch.set_facecolor("#F7F9FC")
    ax.set_facecolor("#F7F9FC")

    end_x    = max(total_dist, 0.1)
    pipe_y   = 0.5
    pipe_top = 0.75
    pipe_bot = 0.25

    # Pipe body
    ax.fill_between([0, end_x], pipe_bot, pipe_top,
                    color="#DDEEFF", alpha=0.6, zorder=1)
    ax.plot([0, end_x], [pipe_top, pipe_top], color="#5599CC", lw=2, zorder=2)
    ax.plot([0, end_x], [pipe_bot, pipe_bot], color="#5599CC", lw=2, zorder=2)
    ax.plot([0, end_x], [pipe_y,   pipe_y  ], color="#AAC8E0",
            lw=1, linestyle="--", zorder=2)

    # Events
    for ev in snap:
        x = ev["distance"]
        if ev["event"] == "CRACK":
            ax.plot(x, pipe_y, "ro", ms=14, zorder=5,
                    mec="#990000", mew=1.5)
            ax.text(x, pipe_top + 0.06, f"{x:.1f}m",
                    ha="center", va="bottom", fontsize=7,
                    color="#990000", rotation=45)
        elif ev["event"] == "BLOCK":
            rect = mpatches.FancyBboxPatch(
                (x - 0.04, pipe_bot + 0.02), 0.08, pipe_top - pipe_bot - 0.04,
                boxstyle="square,pad=0", lw=1.5,
                ec="black", fc="#444444", zorder=5,
            )
            ax.add_patch(rect)
            ax.text(x, pipe_top + 0.06, f"{x:.1f}m",
                    ha="center", va="bottom", fontsize=7,
                    color="#222222", rotation=45)

    # Robot position triangle
    ax.plot(total_dist, pipe_y, "^", ms=12, color="#FF8800",
            mec="#994400", mew=1.5, zorder=6, label="Robot")

    ax.set_xlim(-0.05 * end_x, end_x * 1.10)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlabel("Distance (m)", fontsize=10)
    ax.set_title("Pipeline Inspection Map", fontsize=13, fontweight="bold")

    legend_elems = [
        mpatches.Patch(fc="red",    ec="#990000", label="Crack"),
        mpatches.Patch(fc="#444",   ec="black",   label="Block"),
        plt.Line2D([0], [0], marker="^", color="w",
                   mfc="#FF8800", mec="#994400", ms=10, label="Robot"),
    ]
    ax.legend(handles=legend_elems, loc="upper right", fontsize=9)
    plt.tight_layout()

    # Capture as bytes
    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    buf.seek(0)
    png_bytes = buf.read()
    plt.close(fig)

    if save_file:
        FILE_MAP.write_bytes(png_bytes)
        logger.info("Pipeline map saved: %s", FILE_MAP)

    return png_bytes


def generate_pipeline_map() -> None:
    """Public wrapper — builds map, saves to disk, updates dashboard cache."""
    global map_png_bytes
    png = _build_map_figure(save_file=True)
    with map_png_lock:
        map_png_bytes = png


# ---------------------------------------------------------------------------
# PDF report
# ---------------------------------------------------------------------------

def generate_pdf_report() -> None:
    from reportlab.lib          import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles   import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units    import cm, mm
    from reportlab.platypus     import (
        Image, PageBreak, Paragraph, SimpleDocTemplate,
        Spacer, Table, TableStyle,
    )

    logger.info("Generating PDF report …")

    with distance_lock:
        total_dist = current_distance
    with counter_lock:
        n_cracks = crack_count
        n_blocks = block_count
    with event_log_lock:
        snap = list(event_log)

    ts_str = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")

    doc = SimpleDocTemplate(
        str(FILE_REPORT), pagesize=A4,
        leftMargin=2*cm, rightMargin=2*cm,
        topMargin=2*cm,  bottomMargin=2*cm,
        title="Pipeline Inspection Report",
    )

    styles = getSampleStyleSheet()
    s_title   = ParagraphStyle("T",  parent=styles["Title"],
                                fontSize=22, spaceAfter=6)
    s_sub     = ParagraphStyle("S",  parent=styles["Normal"],
                                fontSize=11, textColor=colors.grey,
                                spaceAfter=16)
    s_heading = ParagraphStyle("H",  parent=styles["Heading2"],
                                fontSize=13, spaceBefore=14, spaceAfter=6,
                                textColor=colors.HexColor("#1A3A5C"))
    s_caption = ParagraphStyle("C",  parent=styles["Normal"],
                                fontSize=9, textColor=colors.grey,
                                alignment=1)

    story = []
    story.append(Paragraph("Pipeline Inspection Report", s_title))
    story.append(Paragraph(f"Generated: {ts_str}", s_sub))
    story.append(Spacer(1, 6*mm))

    story.append(Paragraph("Inspection Summary", s_heading))
    tdata = [
        ["Parameter",             "Value"],
        ["Date / Time",           ts_str],
        ["Total Distance",        f"{total_dist:.2f} m"],
        ["Cracks Detected",       str(n_cracks)],
        ["Blocks Detected",       str(n_blocks)],
    ]
    tbl = Table(tdata, colWidths=[9*cm, 8*cm])
    tbl.setStyle(TableStyle([
        ("BACKGROUND",    (0,0),(-1,0), colors.HexColor("#1A3A5C")),
        ("TEXTCOLOR",     (0,0),(-1,0), colors.white),
        ("FONTNAME",      (0,0),(-1,0), "Helvetica-Bold"),
        ("ROWBACKGROUNDS",(0,1),(-1,-1),
         [colors.HexColor("#EDF3FA"), colors.white]),
        ("GRID",          (0,0),(-1,-1), 0.5, colors.lightgrey),
        ("LEFTPADDING",   (0,0),(-1,-1), 8),
        ("RIGHTPADDING",  (0,0),(-1,-1), 8),
        ("TOPPADDING",    (0,0),(-1,-1), 5),
        ("BOTTOMPADDING", (0,0),(-1,-1), 5),
    ]))
    story.append(tbl)
    story.append(Spacer(1, 8*mm))

    story.append(Paragraph("Pipeline Distance Map", s_heading))
    if FILE_MAP.exists():
        img = cv2.imread(str(FILE_MAP))
        if img is not None:
            h_px, w_px = img.shape[:2]
            max_w = 16*cm
            rpt_img = Image(str(FILE_MAP),
                            width=max_w, height=max_w * h_px / w_px)
            story.append(rpt_img)
            story.append(Paragraph(
                "Figure 1 — Pipeline map. Red circles = cracks; "
                "black squares = blocks; orange triangle = robot.",
                s_caption))
    story.append(Spacer(1, 8*mm))

    crack_evs = [e for e in snap
                 if e["event"] == "CRACK" and Path(e["image"]).exists()]
    if crack_evs:
        story.append(PageBreak())
        story.append(Paragraph("Crack Image Gallery", s_heading))
        story.append(Spacer(1, 4*mm))
        for idx, ev in enumerate(crack_evs, 1):
            p = Path(ev["image"])
            if not p.exists():
                continue
            img = cv2.imread(str(p))
            if img is None:
                continue
            h_px, w_px = img.shape[:2]
            max_w = 10*cm
            story.append(Image(str(p), width=max_w,
                               height=max_w * h_px / w_px))
            story.append(Paragraph(
                f"Figure {idx+1} — Crack detected at "
                f"{ev['distance']:.2f} m  ({ev['timestamp']})",
                s_caption))
            story.append(Spacer(1, 6*mm))
    else:
        story.append(Paragraph("No crack images recorded.", styles["Normal"]))

    doc.build(story)
    logger.info("PDF report saved: %s", FILE_REPORT)


# ---------------------------------------------------------------------------
# Flask dashboard  — Thread 3
# ---------------------------------------------------------------------------

# Inline HTML template (no external files needed — fully offline)
_DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pipeline Inspection Dashboard</title>
<style>
  :root {
    --bg:      #0d1117;
    --surface: #161b22;
    --border:  #30363d;
    --accent:  #58a6ff;
    --warn:    #f78166;
    --ok:      #3fb950;
    --text:    #c9d1d9;
    --muted:   #8b949e;
    --card-r:  10px;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif;
    font-size: 14px;
  }
  header {
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    padding: 14px 24px;
    display: flex; align-items: center; gap: 12px;
  }
  header h1 { font-size: 18px; font-weight: 700; letter-spacing: .5px; }
  header span { font-size: 11px; color: var(--muted); margin-left: auto; }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: 14px;
    padding: 18px 24px 0;
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--card-r);
    padding: 16px;
  }
  .card .label { font-size: 11px; color: var(--muted);
                 text-transform: uppercase; letter-spacing: .8px; }
  .card .value { font-size: 28px; font-weight: 700;
                 margin-top: 6px; color: var(--accent); }
  .card.warn  .value { color: var(--warn); }
  .card.ok    .value { color: var(--ok);   }
  .status-badge {
    display: inline-block;
    padding: 4px 12px; border-radius: 20px;
    font-size: 13px; font-weight: 600;
    margin-top: 8px;
  }
  .MOVING    { background:#1a3a20; color: var(--ok);   }
  .REVERSING { background:#3a2010; color:#e3b341; }
  .STOPPED   { background:#2d1b1b; color: var(--warn); }
  .section {
    margin: 18px 24px 0;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--card-r);
    padding: 14px;
  }
  .section h2 {
    font-size: 13px; font-weight: 700;
    text-transform: uppercase; letter-spacing: .8px;
    color: var(--muted); margin-bottom: 10px;
  }
  #map-img { width:100%; border-radius:6px; display:block; }
  #log-table {
    width:100%; border-collapse:collapse; font-size:12px;
  }
  #log-table th {
    text-align:left; color:var(--muted); font-weight:600;
    padding: 4px 8px; border-bottom: 1px solid var(--border);
  }
  #log-table td { padding: 5px 8px; border-bottom:1px solid #21262d; }
  #log-table tr:last-child td { border:none; }
  .ev-CRACK    { color: var(--warn); font-weight:600; }
  .ev-BLOCK    { color: #e3b341;     font-weight:600; }
  .ev-REVERSING{ color: var(--muted);}
  #crack-img { max-width:260px; border-radius:6px; display:block; }
  footer {
    text-align:center; color:var(--muted); font-size:11px;
    padding: 18px; margin-top:18px;
  }
  .pulse { animation: pulse 1.6s ease-in-out infinite; }
  @keyframes pulse {
    0%,100% { opacity:1; } 50% { opacity:.5; }
  }
</style>
</head>
<body>
<header>
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none"
       stroke="#58a6ff" stroke-width="2" stroke-linecap="round">
    <circle cx="12" cy="12" r="10"/><path d="M8 12h8M12 8l4 4-4 4"/>
  </svg>
  <h1>Pipeline Inspection Dashboard</h1>
  <span id="ts">—</span>
</header>

<div class="grid">
  <div class="card">
    <div class="label">Distance</div>
    <div class="value" id="dist">—</div>
    <div style="color:var(--muted);font-size:11px">metres</div>
  </div>
  <div class="card" id="status-card">
    <div class="label">Robot Status</div>
    <div class="status-badge STOPPED" id="status-badge">STOPPED</div>
  </div>
  <div class="card warn">
    <div class="label">Cracks</div>
    <div class="value" id="cracks">0</div>
  </div>
  <div class="card" style="border-color:#30363d">
    <div class="label">Blocks</div>
    <div class="value" id="blocks">0</div>
  </div>
</div>

<div class="section">
  <h2>Live Pipeline Map</h2>
  <img id="map-img" src="/map_image" alt="pipeline map">
</div>

<div style="display:grid;grid-template-columns:1fr 1fr;gap:14px;
            margin:18px 24px 0;">
  <div class="section" style="margin:0">
    <h2>Detection Log</h2>
    <table id="log-table">
      <thead>
        <tr>
          <th>Dist (m)</th><th>Event</th><th>Time</th>
        </tr>
      </thead>
      <tbody id="log-body"></tbody>
    </table>
  </div>
  <div class="section" style="margin:0">
    <h2>Latest Crack Image</h2>
    <img id="crack-img" src="" alt="No crack image yet"
         style="display:none">
    <p id="no-crack" style="color:var(--muted);font-size:12px">
      No crack detected yet.</p>
  </div>
</div>

<footer>PipelineInspect v2.2 &nbsp;|&nbsp; Offline &nbsp;|&nbsp;
  Auto-refresh every 1 s
  <span class="pulse"> ●</span>
</footer>

<script>
  async function refresh() {
    try {
      const r = await fetch('/api/state');
      const d = await r.json();

      document.getElementById('ts').textContent =
        new Date().toLocaleTimeString();
      document.getElementById('dist').textContent =
        parseFloat(d.distance).toFixed(2);
      document.getElementById('cracks').textContent = d.cracks;
      document.getElementById('blocks').textContent = d.blocks;

      const badge = document.getElementById('status-badge');
      badge.textContent  = d.status;
      badge.className    = 'status-badge ' + d.status;

      // Log table (most-recent first, max 20 rows)
      const tbody = document.getElementById('log-body');
      tbody.innerHTML = '';
      const rows = d.events.slice().reverse().slice(0, 20);
      rows.forEach(ev => {
        const tr = document.createElement('tr');
        tr.innerHTML =
          `<td>${parseFloat(ev.distance).toFixed(2)}</td>` +
          `<td class="ev-${ev.event}">${ev.event}</td>` +
          `<td>${ev.timestamp.split('T')[1] || ev.timestamp}</td>`;
        tbody.appendChild(tr);
      });

      // Map image (cache-bust)
      document.getElementById('map-img').src =
        '/map_image?t=' + Date.now();

      // Latest crack image
      if (d.has_crack_image) {
        document.getElementById('crack-img').src =
          '/latest_crack?t=' + Date.now();
        document.getElementById('crack-img').style.display = 'block';
        document.getElementById('no-crack').style.display  = 'none';
      }
    } catch(e) { /* network hiccup — ignore */ }
  }
  refresh();
  setInterval(refresh, 1000);
</script>
</body>
</html>
"""

app = Flask(__name__)
app.logger.disabled = True          # suppress Flask's own access logs

# Silence Werkzeug request noise to keep the inspection log readable
import logging as _logging
_logging.getLogger("werkzeug").setLevel(_logging.ERROR)


@app.route("/")
def dashboard():
    return render_template_string(_DASHBOARD_HTML)


@app.route("/api/state")
def api_state():
    """JSON snapshot of all dashboard data — polled every second."""
    with distance_lock:
        dist = current_distance
    with status_lock:
        status = robot_status
    with counter_lock:
        nc = crack_count
        nb = block_count
    with event_log_lock:
        snap = list(event_log)
    with latest_crack_lock:
        has_img = latest_crack_jpg is not None

    return jsonify({
        "distance":       dist,
        "status":         status,
        "cracks":         nc,
        "blocks":         nb,
        "events":         snap,
        "has_crack_image": has_img,
    })


@app.route("/map_image")
def map_image():
    """Serve the live pipeline map PNG."""
    # Rebuild lightweight map for dashboard on every request
    try:
        png = _build_map_figure(save_file=False)
    except Exception as exc:
        logger.warning("Map render error: %s", exc)
        # Return a tiny transparent PNG on failure
        png = (
            b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01'
            b'\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00\x00'
            b'\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00\x05\x18'
            b'\xd8N\x00\x00\x00\x00IEND\xaeB`\x82'
        )
    return send_file(io.BytesIO(png), mimetype="image/png",
                     max_age=0)


@app.route("/latest_crack")
def latest_crack():
    """Serve the most recent crack JPEG."""
    with latest_crack_lock:
        jpg = latest_crack_jpg
    if jpg is None:
        return ("No image", 404)
    return send_file(io.BytesIO(jpg), mimetype="image/jpeg", max_age=0)


def start_dashboard() -> None:
    """Thread 3: run Flask on all interfaces, port 5000."""
    logger.info("Dashboard starting at http://0.0.0.0:%d", DASHBOARD_PORT)
    app.run(host="0.0.0.0", port=DASHBOARD_PORT,
            debug=False, use_reloader=False, threaded=True)


# ---------------------------------------------------------------------------
# Thread 1 — Serial listener
# ---------------------------------------------------------------------------

def serial_listener() -> None:
    global current_distance, robot_status

    logger.info("Serial listener started")
    hb_last = time.monotonic()

    while not inspection_done.is_set():
        now = time.monotonic()
        if now - hb_last >= HB_INTERVAL:
            send_command("HB")
            hb_last = now

        if ser is None or not ser.is_open:
            time.sleep(0.5)
            continue

        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            logger.error("Serial read error: %s", exc)
            time.sleep(1.0)
            continue

        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue

        if not line:
            continue

        logger.debug("RX ← ESP32: %s", line)

        # ── v2.2: DATA telemetry packet ─────────────────────────────────────
        # Format: DATA,timestamp,x,y,angle,gyro,speed,distance,crack_flag
        # Example: DATA,197309,0.0000,0.0000,-239.80,2.92,4,400,0
        if line.startswith("DATA,"):
            parts = line.split(",")
            if len(parts) != 9:
                logger.warning(
                    "Malformed DATA packet (expected 9 fields, got %d): %s",
                    len(parts), line,
                )
            else:
                try:
                    ts         = int(parts[1])
                    x          = float(parts[2])
                    y          = float(parts[3])
                    angle      = float(parts[4])
                    gyro       = float(parts[5])
                    speed      = int(parts[6])
                    raw_dist   = int(parts[7])
                    crack_flag = int(parts[8])

                    # Convert raw encoder ticks → metres
                    # Adjust the divisor to match your encoder calibration.
                    dist_m = raw_dist / 1000.0

                    # Update primary distance (used by all other subsystems)
                    with distance_lock:
                        current_distance = dist_m

                    # Mark robot as moving whenever fresh telemetry arrives
                    with status_lock:
                        robot_status = "MOVING"

                    # Persist full telemetry for any future consumers
                    with telemetry_lock:
                        robot_telemetry.update({
                            "timestamp":  ts,
                            "x":          x,
                            "y":          y,
                            "angle":      angle,
                            "gyro":       gyro,
                            "speed":      speed,
                            "distance":   raw_dist,
                            "crack_flag": crack_flag,
                        })

                    logger.debug(
                        "DATA ← ts=%d  pos=(%.3f, %.3f)  angle=%.2f°  "
                        "gyro=%.2f  speed=%d  dist=%d (%.3f m)  crack=%d",
                        ts, x, y, angle, gyro, speed, raw_dist, dist_m,
                        crack_flag,
                    )

                    # If the ESP32 itself flagged a crack, honour it
                    if crack_flag:
                        log_event("CRACK")

                except (ValueError, IndexError) as exc:
                    logger.warning("DATA parse error (%s): %s", exc, line)

        # ── Legacy distance-only message — kept for backwards compatibility ─
        elif line.startswith("DIST:"):
            try:
                val = float(line.split(":")[1])
                with distance_lock:
                    current_distance = val
                with status_lock:
                    robot_status = "MOVING"
            except (IndexError, ValueError) as e:
                logger.warning("Bad DIST '%s': %s", line, e)

        elif line == "BLOCK":
            log_event("BLOCK")
            send_command("STOP")
            with status_lock:
                robot_status = "STOPPED"

        elif line == "REVERSING":
            log_event("REVERSING")
            with status_lock:
                robot_status = "REVERSING"

        elif line == "CLEAR":
            logger.info("Path clear — resuming")
            send_command("MOVE")
            with status_lock:
                robot_status = "MOVING"

        elif line == "END":
            logger.info("END received — finishing inspection")
            send_command("STOP")
            with status_lock:
                robot_status = "STOPPED"
            inspection_done.set()

        else:
            logger.debug("Unknown ESP32 message: %s", line)

    logger.info("Serial listener stopped")


# ---------------------------------------------------------------------------
# UART helper
# ---------------------------------------------------------------------------

def send_command(cmd: str) -> None:
    if ser is None or not ser.is_open:
        logger.warning("Serial not open — cannot send: %s", cmd)
        return
    try:
        ser.write(f"{cmd}\n".encode())
        logger.debug("TX → ESP32: %s", cmd)
    except serial.SerialException as exc:
        logger.error("Serial write error: %s", exc)


# ---------------------------------------------------------------------------
# Thread 2 — Camera + AI
# ---------------------------------------------------------------------------

def process_camera(cap, interpreter, input_size, output_index) -> None:
    global latest_crack_jpg

    logger.info("Camera processing thread started")
    frame_counter = 0

    while not inspection_done.is_set():
        ret, frame = cap.read()
        if not ret:
            logger.warning("Frame capture failed — retrying")
            time.sleep(0.1)
            continue

        frame_counter += 1
        if frame_counter % FRAME_SKIP != 0:
            continue

        score = detect_cracks(frame, interpreter, input_size, output_index)

        if score >= CRACK_THRESHOLD:
            with distance_lock:
                dist = current_distance

            ts_str   = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_name = f"crack_{dist:.2f}_{ts_str}.jpg"
            img_path = DIR_CRACK_IMAGES / img_name

            cv2.imwrite(str(img_path), frame)
            logger.info("Crack @ %.2f m (score=%.3f) → %s", dist, score, img_path)
            log_event("CRACK", str(img_path))

            # Cache JPEG bytes for dashboard
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with latest_crack_lock:
                    latest_crack_jpg = buf.tobytes()

        time.sleep(0.02)   # yield CPU

    logger.info("Camera thread stopped")
    cap.release()


# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------

def handle_sigint(signum, frame) -> None:
    logger.warning("SIGINT — shutting down …")
    inspection_done.set()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    global ser

    setup_logging()
    logger.info("=== Pipeline Inspection Robot v2.2 starting ===")
    signal.signal(signal.SIGINT, handle_sigint)

    initialise_directories()
    initialise_csv()

    # Open serial
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD,
                            timeout=SERIAL_TIMEOUT, write_timeout=1.0)
        logger.info("Serial: %s @ %d baud", SERIAL_PORT, SERIAL_BAUD)
    except serial.SerialException as exc:
        logger.critical("Serial open failed: %s", exc)
        return 1

    # Open camera
    try:
        cap = initialize_camera()
    except RuntimeError as exc:
        logger.critical("Camera: %s", exc)
        ser.close()
        return 1

    # Load model
    try:
        interpreter, input_size, output_index = load_model()
    except FileNotFoundError as exc:
        logger.critical("Model: %s", exc)
        cap.release()
        ser.close()
        return 1

    # Kick off inspection
    send_command("START")
    logger.info("START sent — inspection underway")

    # Thread 1 — Serial listener
    t1 = threading.Thread(target=serial_listener,
                          name="SerialListener", daemon=True)
    # Thread 2 — Camera / AI
    t2 = threading.Thread(target=process_camera,
                          args=(cap, interpreter, input_size, output_index),
                          name="CameraProcessor", daemon=True)
    # Thread 3 — Flask dashboard
    t3 = threading.Thread(target=start_dashboard,
                          name="Dashboard", daemon=True)

    t3.start()   # start dashboard first so it's ready immediately
    t1.start()
    t2.start()

    logger.info(
        "All threads running. Dashboard → http://0.0.0.0:%d  "
        "Waiting for END …", DASHBOARD_PORT
    )

    # Block until inspection finishes (END or Ctrl-C)
    inspection_done.wait()

    logger.info("Inspection done — joining threads …")
    t1.join(timeout=3.0)
    t2.join(timeout=5.0)
    # t3 (Flask) is daemon — it exits with the process

    # Generate outputs
    logger.info("Generating pipeline map …")
    try:
        generate_pipeline_map()
    except Exception as exc:
        logger.error("Map generation error: %s", exc)

    logger.info("Generating PDF report …")
    try:
        generate_pdf_report()
    except Exception as exc:
        logger.error("PDF generation error: %s", exc)

    send_command("STOP")
    if ser.is_open:
        ser.close()

    logger.info(
        "=== Complete | %.2f m | %d cracks | %d blocks ===",
        current_distance, crack_count, block_count,
    )
    logger.info("Map    → %s", FILE_MAP)
    logger.info("Report → %s", FILE_REPORT)
    logger.info("CSV    → %s", FILE_CSV)
    return 0


if __name__ == "__main__":
    sys.exit(main())
