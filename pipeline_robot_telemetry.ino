/**
 * ═══════════════════════════════════════════════════════════════════════════
 * ESP32 PIPELINE INSPECTION ROBOT - MAXIMUM SAFETY HARDENED VERSION
 * + RPi SERIAL TELEMETRY / 2D PATH GENERATION EXTENSION
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Target: ESP32 Dev Module (Arduino Core 3.x)
 * Motor Driver: BTS7960 H-Bridge (high-current dual half-bridge)
 *
 * CRITICAL SAFETY PHILOSOPHY:
 * This robot operates in confined spaces where hardware failure or runaway
 * behavior could cause damage, injury, or mission-critical failure. Every
 * line of code assumes hostile conditions: sensor glitches, power transients,
 * EMI, stuck motors, and unpredictable terrain.
 *
 * DEFENSE LAYERS:
 * 1. Global motor enable gate (volatile, checked every motor command)
 * 2. State machine validation (invalid states trigger emergency stop)
 * 3. Sensor failure handling (timeouts, median filtering, sanity checks)
 * 4. Current monitoring with hysteresis (prevents false trips)
 * 5. Thermal management (mandatory cooling cycles)
 * 6. Stall detection (angle-based timeout)
 * 7. Absolute time limits (mission and continuous run)
 * 8. Hardware-safe PWM sequencing (prevents shoot-through)
 *
 * TELEMETRY EXTENSION (ADDITIVE ONLY - no existing logic modified):
 * - Yaw angle tracking via MPU6050 gyro Z integration
 * - RPM-based odometry (totalDistanceMeters, deltaDistanceMeters)
 * - 200ms serial telemetry to Raspberry Pi for 2D path reconstruction
 * - Format: DATA,<ms>,<delta_m>,<total_m>,<yaw_deg>,<pitch_deg>,<state>,<cm>,<i_raw>
 * - Telemetry output: UART2 (GPIO16=RX, GPIO17=TX, 115200 8N1)
 * - Debug output: Serial / UART0 (USB) unchanged
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <MPU6050.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE UART2 - RASPBERRY PI TELEMETRY CHANNEL
 * TX = GPIO17, RX = GPIO16, 115200 8N1
 * All DATA telemetry lines are sent through this port only.
 * USB Serial (UART0) is preserved exclusively for human-readable debug output.
 * ═══════════════════════════════════════════════════════════════════════════ */

HardwareSerial PiSerial(2);

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE PIN ASSIGNMENTS
 * All pins selected to avoid ESP32 bootstrap pins (0, 2, 4, 5, 12, 14, 15)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define RPWM_PIN 25        // BTS7960 Forward PWM
#define LPWM_PIN 26        // BTS7960 Reverse PWM
#define R_EN_PIN 32        // BTS7960 Forward Enable
#define L_EN_PIN 33        // BTS7960 Reverse Enable

#define TRIG_PIN 18        // HC-SR04 Trigger
#define ECHO_PIN 19        // HC-SR04 Echo
#define BUZZER_PIN 23      // Alert buzzer
#define SDA_PIN 21         // I2C Data (MPU6050)
#define SCL_PIN 22         // I2C Clock (MPU6050)
  
// Optional hardware current sensing (BTS7960 IS pin → ESP32 ADC)
#define CURRENT_SENSE_PIN 34      // ADC1_CH6 (safe, won't conflict with WiFi)
#define USE_CURRENT_SENSING true  // Set false if IS pin not physically connected

/* ═══════════════════════════════════════════════════════════════════════════
 * PWM CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */

#define PWM_FREQ 5000      // 5kHz - thermal safe, low EMI, above audible range
#define PWM_RESOLUTION 8   // 8-bit (0-255)
#define PWM_MAX 255

/* ═══════════════════════════════════════════════════════════════════════════
 * SAFETY PARAMETERS - TUNED FOR MAXIMUM HARDWARE PROTECTION
 * ═══════════════════════════════════════════════════════════════════════════ */

// Soft Start - Prevents inrush current damage to driver and battery
#define SOFT_START_DURATION_MS 2000   // 2 second ramp prevents current spike
#define SOFT_START_MAX_PWM 120        // Conservative startup limit

// Thermal Protection - Prevents continuous heating damage
#define THERMAL_BREAK_INTERVAL_MS 45000   // Mandatory rest every 45 seconds
#define THERMAL_BREAK_DURATION_MS 3000    // 3 second cooling period

// Stall Detection - Protects motor from locked-rotor burnout
#define EXTREME_ANGLE_THRESHOLD_DEG 55.0  // Angles >55° are potentially dangerous
#define EXTREME_ANGLE_TIMEOUT_MS 4000     // Max time on extreme angle before abort
#define MAX_CONTINUOUS_RUN_MS 120000      // 2 minute absolute limit per forward cycle

// Speed Limits - Conservative to prevent mechanical/electrical damage
#define ABSOLUTE_MAX_SPEED 200     // Hard limit enforced in all conditions
#define SPEED_FLAT 150             // Normal cruising
#define SPEED_UPHILL 180           // Increased torque for climbing
#define SPEED_DOWNHILL 120         // Reduced for control
#define SPEED_RETURN 110           // Conservative return speed

// Terrain Angle Detection
#define UPHILL_THRESHOLD_DEG 8.0
#define DOWNHILL_THRESHOLD_DEG -8.0

// Overcurrent Protection - Prevents driver/motor damage from stall or jam
#define OVERCURRENT_THRESHOLD 900       // ADC value (tune for your specific motor)
#define OVERCURRENT_CONFIRM_COUNT 3     // Must exceed threshold this many consecutive times
#define ADC_SAMPLES 5                   // Average this many readings for noise immunity

/* ═══════════════════════════════════════════════════════════════════════════
 * OBSTACLE DETECTION PARAMETERS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define OBSTACLE_DISTANCE_CM 15             // Trigger distance
#define OBSTACLE_CONFIRM_TIME_MS 2000       // Must persist to avoid false positives
#define ULTRASONIC_TIMEOUT_US 20000         // 20ms max (reduced from 30ms for safety)
#define ULTRASONIC_READINGS 3               // Median filter sample count

/* ═══════════════════════════════════════════════════════════════════════════
 * MISSION LIMITS - ABSOLUTE HARD STOPS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MAX_MISSION_DURATION_MS 300000  // 5 minute absolute mission timeout

/* ═══════════════════════════════════════════════════════════════════════════
 * ODOMETRY CONFIGURATION
 * Set these values to match your physical robot hardware.
 * WHEEL_DIAMETER_M : measured outer diameter of drive wheel in metres.
 * MOTOR_RPM        : no-load shaft RPM at your nominal operating voltage.
 *                    Measure with a tachometer or derive from motor datasheet.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define WHEEL_DIAMETER_M  0.10    // SET YOUR REAL VALUE (metres)
#define MOTOR_RPM         30.0   // SET YOUR REAL NO-LOAD RPM

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE DEFINITION
 * ═══════════════════════════════════════════════════════════════════════════ */

enum RobotState {
  STATE_FORWARD,          // Active exploration
  STATE_THERMAL_BREAK,    // Mandatory cooling rest
  STATE_FAILSAFE_RETURN,  // Reversing to start
  STATE_EMERGENCY_STOP,   // Safety halt - requires reset
  STATE_COMPLETED         // Mission success
};

/* ═══════════════════════════════════════════════════════════════════════════
 * GLOBAL STATE - VOLATILE WHERE MODIFIED BY ISR OR SAFETY CHECKS
 * ═══════════════════════════════════════════════════════════════════════════ */

// CRITICAL: Global motor enable gate
// This flag provides absolute control over motor operation
// Once set false by emergencyStop(), motors CANNOT run until reset
volatile bool motorEnableGate = true;

MPU6050 mpu;
RobotState currentState = STATE_FORWARD;

// Timing trackers (millisecond precision)
unsigned long forwardStartTime = 0;
unsigned long thermalBreakStartTime = 0;
unsigned long lastThermalBreakTime = 0;
unsigned long continuousRunStartTime = 0;
unsigned long extremeAngleStartTime = 0;
unsigned long obstacleDetectedTime = 0;
unsigned long reverseStartTime = 0;
unsigned long lastStatusPrintTime = 0;

// State flags
bool wasOnExtremeAngle = false;
bool obstacleConfirming = false;

// Sensor data (filtered)
float pitchAngleDeg = 0.0;
int distanceCm = 0;

// Safety counters
int overcurrentCounter = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * TELEMETRY / ODOMETRY GLOBALS
 * Added for ESP32 → Raspberry Pi 2D path telemetry.
 * These variables are written only by updateOdometry() and
 * calculateYawAngle(), and read only by sendPathTelemetry().
 * They do not interact with any safety, motor, or state-machine logic.
 * ═══════════════════════════════════════════════════════════════════════════ */

float         yawAngleDeg         = 0.0;   // Integrated gyro-Z heading (degrees)
float         totalDistanceMeters = 0.0;   // Net signed displacement from start (m)
float         deltaDistanceMeters = 0.0;   // Distance travelled in last odometry tick (m)
unsigned long lastOdometryTime    = 0;     // Timestamp of previous odometry update (ms)
unsigned long lastTelemetrySend   = 0;     // Timestamp of last telemetry serial send (ms)

/* ═══════════════════════════════════════════════════════════════════════════
 * ARDUINO SETUP - INITIALIZE HARDWARE TO SAFE STATE
 * ═══════════════════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize UART2 for Raspberry Pi telemetry
  // RX = GPIO16, TX = GPIO17, 115200 baud, 8N1
  PiSerial.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  ESP32 PIPELINE ROBOT - HARDENED SAFETY MODE   ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");

  // Initialize hardware to safe state BEFORE anything else
  initializeHardware();

  // Initialize sensors (halt if critical sensor fails)
  if (!initializeMPU6050()) {
    emergencyStop("MPU6050 initialization failed - CRITICAL SENSOR MISSING");
    while (1) {
      blinkBuzzer(300);
      yield();  // Feed watchdog even in error loop
    }
  }

  printSafetyFeatures();

  Serial.println("\n⚠ CALIBRATION: Place robot on FLAT, STABLE surface");
  Serial.println("   Waiting 3 seconds...\n");
  delay(3000);

  Serial.println("Mission will start in 3 seconds...");
  delay(3000);

  // Initialize mission timers
  forwardStartTime       = millis();
  lastThermalBreakTime   = millis();
  continuousRunStartTime = millis();

  // Initialize odometry timers
  lastOdometryTime  = millis();
  lastTelemetrySend = millis();

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║              MISSION STARTED                   ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ARDUINO LOOP - MAIN CONTROL CYCLE
 * Only additions to the original loop() body:
 *   updateOdometry()    - called after sensor update, before safety checks
 *   sendPathTelemetry() - called after state machine, before delay
 * All original calls and their order are completely preserved.
 * ═══════════════════════════════════════════════════════════════════════════ */

void loop() {
  // Feed watchdog (prevent reset during long operations)
  yield();

  // Update sensor readings
  updateSensors();

  // Update odometry and yaw for telemetry (additive - does not affect safety)
  updateOdometry();

  // CRITICAL: Run all safety checks BEFORE state machine
  // Any safety violation immediately halts motors
  if (!runSafetyChecks()) {
    return;  // Safety check failed - system is halted
  }

  // Execute current state logic
  executeStateMachine();

  // Report status to serial monitor
  printStatus();

  // Send 2D path telemetry to Raspberry Pi
  sendPathTelemetry();

  // 25Hz main loop rate (40ms cycle time)
  delay(40);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE INITIALIZATION
 * ═══════════════════════════════════════════════════════════════════════════ */

void initializeHardware() {
  // Configure all GPIO pins
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (USE_CURRENT_SENSING) {
    pinMode(CURRENT_SENSE_PIN, INPUT);
  }

  // CRITICAL: Ensure motor driver is DISABLED at startup
  // Both half-bridges must be off to prevent undefined state
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize PWM channels (ESP32 Core 3.x API)
  // Returns true on success, false on failure
  if (!ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("ERROR: Failed to initialize RPWM channel");
  }
  if (!ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("ERROR: Failed to initialize LPWM channel");
  }

  // Set both PWM channels to zero (motors stopped)
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);

  Serial.println("✓ Hardware initialized to safe state");
  Serial.println("  - Motor enables: LOW");
  Serial.println("  - PWM channels: 0");
  Serial.println("  - Motor gate: ENABLED");
}

bool initializeMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // Fast I2C (400kHz)

  Serial.print("Initializing MPU6050... ");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("FAILED ✗");
    return false;
  }

  Serial.println("SUCCESS ✓");
  return true;
}

void printSafetyFeatures() {
  Serial.println("═══════════════════════════════════════");
  Serial.println("  ACTIVE SAFETY FEATURES");
  Serial.println("═══════════════════════════════════════");
  Serial.println("  ✓ Global motor enable gate");
  Serial.println("  ✓ State validation (invalid → E-STOP)");
  Serial.println("  ✓ Isolated half-bridge control");
  Serial.println("  ✓ 5kHz PWM (thermal safe)");
  Serial.println("  ✓ 2s soft-start (120 PWM limit)");
  Serial.println("  ✓ Thermal breaks (every 45s)");
  Serial.println("  ✓ Stall detection (>55° timeout)");
  Serial.println("  ✓ 2min continuous run limit");
  Serial.println("  ✓ Absolute max speed: 200 PWM");

  if (USE_CURRENT_SENSING) {
    Serial.println("  ✓ Overcurrent monitoring (averaged)");
  }

  Serial.println("  ✓ Obstacle confirmation (2s filter)");
  Serial.println("  ✓ Ultrasonic timeout: 20ms");
  Serial.println("  ✓ RPM odometry (2D path telemetry)");
  Serial.println("  ✓ Gyro-Z yaw integration (2D path telemetry)");
  Serial.println("  ✓ UART2 telemetry (GPIO16/17 → RPi)");
  Serial.println("═══════════════════════════════════════");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SAFETY CHECK SYSTEM - RUN BEFORE EVERY STATE MACHINE CYCLE
 * ═══════════════════════════════════════════════════════════════════════════ */

bool runSafetyChecks() {
  // Safety Check 1: Overcurrent detection (if hardware available)
  if (!checkOvercurrent()) {
    emergencyStop("OVERCURRENT - Possible motor stall or jam");
    return false;
  }

  // Safety Check 2: Extreme angle timeout (prevents locked-rotor damage)
  if (!checkExtremeAngleTimeout()) {
    emergencyStop("STALL SUSPECTED - Extreme angle timeout exceeded");
    return false;
  }

  // Safety Check 3: Continuous forward run time limit
  if (currentState == STATE_FORWARD) {
    if (millis() - continuousRunStartTime > MAX_CONTINUOUS_RUN_MS) {
      emergencyStop("CONTINUOUS RUN TIMEOUT - 2 minute limit exceeded");
      return false;
    }
  }

  // Safety Check 4: Absolute mission duration limit
  if (currentState == STATE_FORWARD) {
    if (millis() - forwardStartTime > MAX_MISSION_DURATION_MS) {
      emergencyStop("MISSION TIMEOUT - 5 minute absolute limit exceeded");
      return false;
    }
  }

  return true;  // All checks passed
}

bool checkOvercurrent() {
  // Only monitor current during forward movement (not during breaks or return)
  if (!USE_CURRENT_SENSING || currentState != STATE_FORWARD) {
    // CRITICAL: Reset counter when not actively monitoring
    // Prevents false positives after state transitions
    overcurrentCounter = 0;
    return true;
  }

  // Average multiple ADC readings for noise immunity
  // Single readings can spike due to PWM switching, EMI, or supply noise
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(CURRENT_SENSE_PIN);
    delayMicroseconds(100);  // Small delay between samples
    yield();  // Feed watchdog during ADC sampling
  }
  int avgCurrent = sum / ADC_SAMPLES;

  if (avgCurrent > OVERCURRENT_THRESHOLD) {
    overcurrentCounter++;

    // Require multiple consecutive high readings to confirm
    // This provides hysteresis and prevents false trips from transients
    if (overcurrentCounter >= OVERCURRENT_CONFIRM_COUNT) {
      Serial.print("\n!!! OVERCURRENT DETECTED !!!");
      Serial.print("\n    Averaged ADC reading: ");
      Serial.println(avgCurrent);
      return false;
    }
  } else {
    // Reset counter on normal reading (hysteresis)
    overcurrentCounter = 0;
  }

  return true;
}

bool checkExtremeAngleTimeout() {
  // Only check during forward movement
  if (currentState != STATE_FORWARD) {
    // Reset flag when not in forward state
    wasOnExtremeAngle = false;
    return true;
  }

  bool isExtremeAngle = (abs(pitchAngleDeg) > EXTREME_ANGLE_THRESHOLD_DEG);

  if (isExtremeAngle) {
    if (!wasOnExtremeAngle) {
      // First detection of extreme angle - start timeout timer
      extremeAngleStartTime = millis();
      wasOnExtremeAngle = true;

      Serial.print("\n⚠ EXTREME ANGLE DETECTED: ");
      Serial.print(pitchAngleDeg, 1);
      Serial.println("°");
      Serial.println("  Monitoring for potential stall...");
    }

    // Check if we've been stuck too long (likely stalled motor)
    if (millis() - extremeAngleStartTime > EXTREME_ANGLE_TIMEOUT_MS) {
      Serial.println("\n!!! EXTREME ANGLE TIMEOUT !!!");
      Serial.println("    Motor likely stalled on steep section");
      return false;
    }
  } else {
    // Returned to safe angle range
    if (wasOnExtremeAngle) {
      Serial.println("✓ Returned to safe angle range");
    }
    wasOnExtremeAngle = false;
  }

  return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * EMERGENCY STOP - PERMANENT MOTOR DISABLE UNTIL RESET
 * ═══════════════════════════════════════════════════════════════════════════ */

void emergencyStop(const char* reason) {
  // CRITICAL: Disable motor gate FIRST
  // This prevents ANY further motor commands from executing
  motorEnableGate = false;

  // Ensure motors are physically stopped
  stopMotors();

  // Update state machine
  currentState = STATE_EMERGENCY_STOP;

  // Alert pattern: double beep
  blinkBuzzer(100);
  delay(100);
  blinkBuzzer(100);

  // Print detailed emergency stop report
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║          EMERGENCY STOP ACTIVATED             ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.print("  Reason: ");
  Serial.println(reason);
  Serial.println("  Motor gate: DISABLED");
  Serial.println("  Motors: STOPPED");
  Serial.println("  ");
  Serial.println("  ⚠ Robot halted for hardware safety");
  Serial.println("  ⚠ Investigate cause before reset");
  Serial.println("╚═══════════════════════════════════════════════╝\n");
}

void blinkBuzzer(int durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SENSOR READING AND FILTERING
 * ═══════════════════════════════════════════════════════════════════════════ */

void updateSensors() {
  distanceCm    = readUltrasonicDistance();
  pitchAngleDeg = calculatePitchAngle();
}

int readUltrasonicDistance() {
  int readings[ULTRASONIC_READINGS];
  int validCount = 0;

  // Collect multiple readings for median filtering
  for (int i = 0; i < ULTRASONIC_READINGS; i++) {
    int dist = singleUltrasonicReading();

    // Sanity check: valid range is 2-400cm for HC-SR04
    if (dist > 0 && dist < 400) {
      readings[validCount++] = dist;
    }

    delay(10);  // Brief delay between pings (required by sensor)
    yield();    // Feed watchdog
  }

  // If ALL readings failed, assume path is clear (fail-safe)
  if (validCount == 0) {
    return 400;
  }

  // Use median filtering for best noise rejection
  if (validCount >= 3) {
    // Simple bubble sort (acceptable for small array)
    for (int i = 0; i < validCount - 1; i++) {
      for (int j = 0; j < validCount - i - 1; j++) {
        if (readings[j] > readings[j + 1]) {
          int temp = readings[j];
          readings[j] = readings[j + 1];
          readings[j + 1] = temp;
        }
      }
    }
    return readings[validCount / 2];  // Return median value
  }

  // Fall back to average if less than 3 valid readings
  long sum = 0;
  for (int i = 0; i < validCount; i++) {
    sum += readings[i];
  }
  return sum / validCount;
}

int singleUltrasonicReading() {
  // Standard HC-SR04 trigger sequence
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo with BOUNDED timeout (critical for safety)
  // Reduced to 20ms max to prevent loop blocking
  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

  // Timeout or invalid reading
  if (duration == 0) {
    return 400;  // Assume clear (fail-safe behavior)
  }

  // Convert echo time to distance: duration(μs) × 0.034 / 2
  // Speed of sound: 343 m/s = 0.0343 cm/μs
  return (duration * 0.034) / 2;
}

float calculatePitchAngle() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw accelerometer values to g's
  // MPU6050 default range: ±2g, sensitivity: 16384 LSB/g
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate pitch angle using accelerometer
  // Pitch = forward/backward tilt (rotation around X-axis)
  // Positive pitch = nose up, Negative pitch = nose down
  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;

  // Apply low-pass filter to reduce noise
  // Coefficient 0.8 provides good balance between responsiveness and stability
  static float filteredPitch = 0.0;
  filteredPitch = 0.8 * filteredPitch + 0.2 * pitch;

  return filteredPitch;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * YAW ANGLE - GYRO Z INTEGRATION
 * Reads raw gyroscope Z-axis from MPU6050 and integrates over time.
 * Called exclusively by updateOdometry(). Not called from updateSensors()
 * to avoid any interaction with the existing pitch/accelerometer path.
 *
 * NOTE: Raw gyro integration accumulates drift over time. This is acceptable
 * for pipeline inspection where the physical path constrains heading changes.
 * For missions longer than ~2 minutes consider a complementary or Madgwick
 * filter. The yaw value is used only for telemetry — it has no influence on
 * any safety check, motor command, or state transition.
 * ═══════════════════════════════════════════════════════════════════════════ */

float calculateYawAngle(float dtSeconds) {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // MPU6050 default gyro range: ±250°/s, sensitivity: 131 LSB/(°/s)
  float gyroZ_dps = gz / 131.0f;

  // Integrate: yaw += ω × Δt
  // Static accumulator persists across calls
  static float yaw = 0.0f;
  yaw += gyroZ_dps * dtSeconds;

  return yaw;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ODOMETRY UPDATE
 * Computes deltaDistanceMeters and totalDistanceMeters using wheel RPM and
 * elapsed time. Signed: positive = forward, negative = reverse.
 * Also updates yawAngleDeg via gyro-Z integration.
 *
 * Called once per loop() iteration, immediately after updateSensors().
 * Reads currentState read-only — does NOT write to it.
 * Does NOT interact with motorEnableGate, PWM, or any safety variable.
 * ═══════════════════════════════════════════════════════════════════════════ */

void updateOdometry() {
  unsigned long now = millis();
  float dtSeconds   = (now - lastOdometryTime) / 1000.0f;
  lastOdometryTime  = now;

  // Guard against very first call or large gaps (e.g. after thermal break)
  if (dtSeconds <= 0.0f || dtSeconds > 1.0f) {
    // Still update yaw with a zero or clamped dt to keep accumulator stable
    yawAngleDeg = calculateYawAngle(0.0f);
    deltaDistanceMeters = 0.0f;
    return;
  }

  // Wheel circumference and no-load linear speed
  const float circumference = PI * WHEEL_DIAMETER_M;             // metres per revolution
  const float linearSpeed   = (MOTOR_RPM * circumference) / 60.0f; // metres per second

  // Accumulate distance only while motors are actively travelling.
  // Thermal breaks and completed/emergency states produce zero delta.
  if (currentState == STATE_FORWARD) {
    deltaDistanceMeters    =  linearSpeed * dtSeconds;
    totalDistanceMeters   +=  deltaDistanceMeters;
  } else if (currentState == STATE_FAILSAFE_RETURN) {
    deltaDistanceMeters    = -(linearSpeed * dtSeconds);
    totalDistanceMeters   +=   deltaDistanceMeters;
    if (totalDistanceMeters < 0.0f) totalDistanceMeters = 0.0f;
  } else {
    // STATE_THERMAL_BREAK, STATE_EMERGENCY_STOP, STATE_COMPLETED
    deltaDistanceMeters = 0.0f;
  }

  // Update integrated yaw regardless of motion state
  yawAngleDeg = calculateYawAngle(dtSeconds);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR CONTROL - HARDWARE-SAFE PWM SEQUENCING
 * ═══════════════════════════════════════════════════════════════════════════ */

void setMotorSpeed(int speed, bool forward) {
  // CRITICAL: Check global motor gate FIRST
  // If gate is disabled (emergency stop), motors MUST NOT run
  if (!motorEnableGate) {
    stopMotors();
    return;
  }

  // Enforce absolute maximum speed limit
  speed = constrain(speed, 0, ABSOLUTE_MAX_SPEED);

  // Apply soft-start limiting during initial ramp-up phase
  speed = applySoftStartLimit(speed);

  // Execute appropriate direction sequence
  if (forward) {
    setMotorForward(speed);
  } else {
    setMotorReverse(speed);
  }
}

int applySoftStartLimit(int requestedSpeed) {
  unsigned long elapsedMs = millis() - forwardStartTime;

  if (elapsedMs < SOFT_START_DURATION_MS) {
    // Linear ramp from 0 to SOFT_START_MAX_PWM over SOFT_START_DURATION_MS
    // This prevents current surge that could damage motor driver or brownout ESP32
    int maxAllowed = map(elapsedMs, 0, SOFT_START_DURATION_MS, 0, SOFT_START_MAX_PWM);
    return min(requestedSpeed, maxAllowed);
  }

  return requestedSpeed;
}

void setMotorForward(int speed) {
  // CRITICAL SAFETY SEQUENCE TO PREVENT SHOOT-THROUGH:
  // Shoot-through occurs when both half-bridges conduct simultaneously,
  // creating a direct short from V+ to GND through the MOSFETs.
  // This can instantly destroy the BTS7960 driver.

  // Step 1: Disable reverse half-bridge
  digitalWrite(L_EN_PIN, LOW);

  // Step 2: Set reverse PWM to zero
  ledcWrite(LPWM_PIN, 0);

  // Step 3: DEAD TIME - Wait for reverse MOSFETs to fully turn off
  // 500μs is conservative and ensures complete turn-off even with gate charge
  delayMicroseconds(500);

  // Step 4: Set forward PWM to desired speed
  ledcWrite(RPWM_PIN, speed);

  // Step 5: Brief stabilization delay
  delayMicroseconds(100);

  // Step 6: Enable forward half-bridge
  digitalWrite(R_EN_PIN, HIGH);
}

void setMotorReverse(int speed) {
  // CRITICAL SAFETY SEQUENCE (mirror of forward)

  // Step 1: Disable forward half-bridge
  digitalWrite(R_EN_PIN, LOW);

  // Step 2: Set forward PWM to zero
  ledcWrite(RPWM_PIN, 0);

  // Step 3: DEAD TIME - Prevents shoot-through
  delayMicroseconds(500);

  // Step 4: Set reverse PWM to desired speed
  ledcWrite(LPWM_PIN, speed);

  // Step 5: Brief stabilization delay
  delayMicroseconds(100);

  // Step 6: Enable reverse half-bridge
  digitalWrite(L_EN_PIN, HIGH);
}

void stopMotors() {
  // CRITICAL: This function MUST guarantee motors are stopped
  // Called in ALL non-forward states and emergency conditions

  // Step 1: Set both PWM channels to zero
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);

  // Step 2: Brief settling time for PWM to propagate
  delay(1);

  // Step 3: Disable both half-bridges
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE EXECUTION
 * ═══════════════════════════════════════════════════════════════════════════ */

void executeStateMachine() {
  switch (currentState) {
    case STATE_FORWARD:
      handleForwardState();
      break;

    case STATE_THERMAL_BREAK:
      handleThermalBreakState();
      break;

    case STATE_FAILSAFE_RETURN:
      handleReturnState();
      break;

    case STATE_EMERGENCY_STOP:
      // Ensure motors remain stopped
      stopMotors();
      break;

    case STATE_COMPLETED:
      // Ensure motors remain stopped
      stopMotors();
      break;

    default:
      // CRITICAL: Invalid state detected
      // This should NEVER happen, but if it does, immediately halt
      emergencyStop("INVALID STATE DETECTED - State machine corrupted");
      break;
  }
}

void handleForwardState() {
  // Check if thermal break is required
  if (millis() - lastThermalBreakTime >= THERMAL_BREAK_INTERVAL_MS) {
    Serial.println("\n─── THERMAL BREAK (mandatory cooling) ───");
    stopMotors();
    thermalBreakStartTime = millis();
    currentState = STATE_THERMAL_BREAK;
    return;
  }

  // Check for obstacle
  if (detectObstacle()) {
    initiateReturn();
    return;
  }

  // Calculate appropriate speed based on terrain angle
  int targetSpeed = calculateAdaptiveSpeed();

  // Drive forward at calculated speed
  setMotorSpeed(targetSpeed, true);
}

bool detectObstacle() {
  bool obstaclePresent = (distanceCm < OBSTACLE_DISTANCE_CM);

  if (obstaclePresent) {
    if (!obstacleConfirming) {
      // First detection - start confirmation timer
      // Prevents false positives from sensor glitches or transient echoes
      obstacleConfirming = true;
      obstacleDetectedTime = millis();

      Serial.print("\n⚠ OBSTACLE DETECTED: ");
      Serial.print(distanceCm);
      Serial.println(" cm - Confirming...");
    }

    // Check if obstacle has persisted for confirmation period
    if (millis() - obstacleDetectedTime >= OBSTACLE_CONFIRM_TIME_MS) {
      Serial.println("✓ Obstacle CONFIRMED (persistent 2+ seconds)");
      return true;
    }
  } else {
    // Obstacle cleared before confirmation period
    if (obstacleConfirming) {
      Serial.println("✓ False alarm - obstacle cleared during confirmation");
      obstacleConfirming = false;
    }
  }

  return false;
}

int calculateAdaptiveSpeed() {
  // Adjust motor speed based on terrain angle
  // This provides consistent forward velocity regardless of slope
  int speed = SPEED_FLAT;

  if (pitchAngleDeg > UPHILL_THRESHOLD_DEG) {
    // Climbing - increase power for torque
    speed = SPEED_UPHILL;

    // Additional safety reduction on very steep climbs
    // Prevents excessive current draw that could damage motor
    if (pitchAngleDeg > 45.0) {
      speed = constrain(speed - 30, 100, SPEED_UPHILL);
    }

  } else if (pitchAngleDeg < DOWNHILL_THRESHOLD_DEG) {
    // Descending - reduce speed for control
    speed = SPEED_DOWNHILL;
  }

  return speed;
}

void initiateReturn() {
  stopMotors();

  // Alert beep
  blinkBuzzer(200);

  unsigned long forwardDuration = millis() - forwardStartTime;

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║      OBSTACLE CONFIRMED - INITIATING RETURN    ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.print("  Forward duration: ");
  Serial.print(forwardDuration / 1000.0, 1);
  Serial.println(" seconds");
  Serial.print("  Final distance to obstacle: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  Serial.println("  Beginning return sequence...\n");

  reverseStartTime = millis();
  currentState = STATE_FAILSAFE_RETURN;
}

void handleThermalBreakState() {
  unsigned long elapsedMs = millis() - thermalBreakStartTime;

  if (elapsedMs >= THERMAL_BREAK_DURATION_MS) {
    Serial.println("✓ Thermal break complete - Resuming\n");

    lastThermalBreakTime   = millis();
    continuousRunStartTime = millis();  // Reset continuous run timer

    // CRITICAL: Reset overcurrent counter after thermal break
    // Prevents false positives from transient current during restart
    overcurrentCounter = 0;

    currentState = STATE_FORWARD;
  } else {
    // MUST ensure motors are stopped during thermal break
    stopMotors();
  }
}

void handleReturnState() {
  unsigned long forwardDuration  = reverseStartTime - forwardStartTime;
  unsigned long reverseElapsed   = millis() - reverseStartTime;

  // Sanity check on forward duration (prevent overflow or unreasonable values)
  if (forwardDuration > MAX_MISSION_DURATION_MS) {
    Serial.println("⚠ Forward duration capped at safety limit");
    forwardDuration = MAX_MISSION_DURATION_MS;
  }

  // Check if we've returned for the same duration as forward travel
  if (reverseElapsed >= forwardDuration) {
    completeMission();
    return;
  }

  // Print progress update every 2 seconds
  static unsigned long lastProgressPrint = 0;
  if (millis() - lastProgressPrint > 2000) {
    float percentComplete = (reverseElapsed * 100.0) / forwardDuration;
    Serial.print("◄ RETURNING: ");
    Serial.print(percentComplete, 1);
    Serial.println("%");
    lastProgressPrint = millis();
  }

  // Drive in reverse at conservative speed
  // Lower speed reduces risk during return journey
  setMotorSpeed(SPEED_RETURN, false);
}

void completeMission() {
  stopMotors();
  digitalWrite(BUZZER_PIN, LOW);

  unsigned long totalDuration = reverseStartTime - forwardStartTime;

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║         MISSION COMPLETED SUCCESSFULLY         ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.print("  Total forward time: ");
  Serial.print(totalDuration / 1000.0, 1);
  Serial.println(" seconds");
  Serial.println("  Robot successfully returned to start");
  Serial.println("  System ready for reset\n");

  currentState = STATE_COMPLETED;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATUS REPORTING
 * ═══════════════════════════════════════════════════════════════════════════ */

void printStatus() {
  // Handle special states with periodic messages
  if (currentState == STATE_EMERGENCY_STOP) {
    static unsigned long lastEmergencyPrint = 0;
    if (millis() - lastEmergencyPrint > 3000) {
      Serial.println("⚠ EMERGENCY STOP ACTIVE - Reset required");
      lastEmergencyPrint = millis();
    }
    return;
  }

  if (currentState == STATE_COMPLETED) {
    static unsigned long lastCompletePrint = 0;
    if (millis() - lastCompletePrint > 5000) {
      Serial.println("✓ Mission complete - Reset to restart");
      lastCompletePrint = millis();
    }
    return;
  }

  // Regular status update every 500ms during active operation
  if (millis() - lastStatusPrintTime > 500) {
    Serial.print("Pitch: ");
    Serial.print(pitchAngleDeg, 1);
    Serial.print("° | Dist: ");
    Serial.print(distanceCm);
    Serial.print("cm | State: ");

    switch (currentState) {
      case STATE_FORWARD:
        Serial.print("FORWARD");
        break;
      case STATE_THERMAL_BREAK:
        Serial.print("THERMAL_BREAK");
        break;
      case STATE_FAILSAFE_RETURN:
        Serial.print("RETURN");
        break;
      default:
        Serial.print("UNKNOWN");
        break;
    }

    // Add current sensing data if available
    if (USE_CURRENT_SENSING) {
      int current = analogRead(CURRENT_SENSE_PIN);
      Serial.print(" | I: ");
      Serial.print(current);
    }

    Serial.println();
    lastStatusPrintTime = millis();
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PATH TELEMETRY - ESP32 → RASPBERRY PI via UART2
 *
 * Sends a CSV line every 200 ms on UART2 (GPIO16=RX, GPIO17=TX, 115200 8N1).
 * USB Serial (UART0) is NOT used here — all PiSerial calls are isolated to
 * this function only. All other Serial.print calls in the firmware remain
 * on USB UART0 and are completely untouched.
 *
 * The Raspberry Pi parser reads only lines beginning with "DATA,".
 *
 * Field layout (all fields on one line, terminated with \n):
 *
 *   DATA , <millis> , <delta_m> , <total_m> , <yaw_deg> ,
 *          <pitch_deg> , <state_int> , <ultrasonic_cm> , <current_raw>
 *
 * Field descriptions:
 *   millis        - ESP32 uptime in milliseconds (unsigned long)
 *   delta_m       - signed distance travelled since last odometry tick (m, 4dp)
 *   total_m       - net signed displacement from start position (m, 4dp)
 *   yaw_deg       - integrated gyro-Z heading from start (degrees, 2dp)
 *   pitch_deg     - low-pass filtered accelerometer pitch (degrees, 2dp)
 *   state_int     - RobotState enum value (0=FWD,1=THERMAL,2=RETURN,3=ESTOP,4=DONE)
 *   ultrasonic_cm - median-filtered HC-SR04 distance (integer cm)
 *   current_raw   - raw ADC value from CURRENT_SENSE_PIN (integer, 0 if disabled)
 *
 * This function does NOT modify any variable except lastTelemetrySend.
 * It does NOT call emergencyStop(), stopMotors(), or any safety function.
 * ═══════════════════════════════════════════════════════════════════════════ */

void sendPathTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetrySend < 200UL) return;
  lastTelemetrySend = now;  // Stamp at start of block to avoid drift

  int currentRaw = USE_CURRENT_SENSING ? analogRead(CURRENT_SENSE_PIN) : 0;

  PiSerial.print("DATA,");
  PiSerial.print(now);
  PiSerial.print(",");
  PiSerial.print(deltaDistanceMeters, 4);
  PiSerial.print(",");
  PiSerial.print(totalDistanceMeters, 4);
  PiSerial.print(",");
  PiSerial.print(yawAngleDeg, 2);
  PiSerial.print(",");
  PiSerial.print(pitchAngleDeg, 2);
  PiSerial.print(",");
  PiSerial.print((int)currentState);
  PiSerial.print(",");
  PiSerial.print(distanceCm);
  PiSerial.print(",");
  PiSerial.println(currentRaw);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * END OF HARDENED ROBOT FIRMWARE + TELEMETRY EXTENSION
 * ═══════════════════════════════════════════════════════════════════════════ */
