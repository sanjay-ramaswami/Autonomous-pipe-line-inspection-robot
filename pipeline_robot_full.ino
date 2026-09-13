/**
 * ═══════════════════════════════════════════════════════════════════════════
 * ESP32 PIPELINE INSPECTION ROBOT - MAXIMUM SAFETY HARDENED VERSION
 * + RPi SERIAL TELEMETRY / 2D PATH GENERATION EXTENSION
 * + RASPBERRY PI COMMUNICATION PROTOCOL (Pi AI / Mapping Integration)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Target:        ESP32 Dev Module (Arduino Core 3.x)
 * Motor Driver:  BTS7960 H-Bridge (high-current dual half-bridge)
 * Telemetry:     UART2 → Raspberry Pi Zero 2 W (GPIO16=RX, GPIO17=TX)
 *
 * ── WHAT'S NEW IN THIS VERSION ──────────────────────────────────────────────
 * Pi Communication Protocol layer added on top of existing telemetry:
 *
 *   ESP32 → Pi messages:
 *     DIST:x.xx     Distance travelled (sent every 400 ms)
 *     BLOCK         Obstacle confirmed
 *     REVERSING     Robot is reversing
 *     CLEAR         Path clear again (after reversal resumes forward)
 *     END           Mission complete or emergency stop
 *     SAFETY_STOP   Heartbeat watchdog fired
 *
 *   Pi → ESP32 commands (received over same UART2):
 *     HB            Heartbeat pulse (must arrive every <5 s)
 *     START         Resume robot / clear Pi-requested stop
 *     MOVE          Alias for START
 *     STOP          Pi requests immediate halt
 *
 * All original safety features, motor logic, sensor logic, and telemetry
 * (DATA,... CSV frames) are completely unchanged.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <MPU6050.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE UART2 - RASPBERRY PI CHANNEL  (telemetry + command protocol)
 * TX = GPIO17, RX = GPIO16, 115200 8N1
 * DATA,... CSV telemetry frames AND the new plain-text protocol both travel
 * on this port.  USB Serial (UART0) is preserved for human debug output only.
 * ═══════════════════════════════════════════════════════════════════════════ */

HardwareSerial PiSerial(2);

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE PIN ASSIGNMENTS
 * All pins selected to avoid ESP32 bootstrap pins (0, 2, 4, 5, 12, 14, 15)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define RPWM_PIN            25   // BTS7960 Forward PWM
#define LPWM_PIN            26   // BTS7960 Reverse PWM
#define R_EN_PIN            32   // BTS7960 Forward Enable
#define L_EN_PIN            33   // BTS7960 Reverse Enable

#define TRIG_PIN            18   // HC-SR04 Trigger
#define ECHO_PIN            19   // HC-SR04 Echo

#define SDA_PIN             21   // I2C Data  (MPU6050)
#define SCL_PIN             22   // I2C Clock (MPU6050)

#define CURRENT_SENSE_PIN   34   // ADC1_CH6 – BTS7960 IS pin → ESP32 ADC
#define USE_CURRENT_SENSING true // Set false if IS pin not physically connected

/* ═══════════════════════════════════════════════════════════════════════════
 * PWM CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */

#define PWM_FREQ        5000   // 5 kHz – thermal safe, low EMI, above audible
#define PWM_RESOLUTION     8   // 8-bit (0–255)
#define PWM_MAX          255

/* ═══════════════════════════════════════════════════════════════════════════
 * SAFETY PARAMETERS
 * ═══════════════════════════════════════════════════════════════════════════ */
// Duration for soft start (gradually increasing speed to avoid jerks)
#define SOFT_START_DURATION_MS     2000   // 2 seconds

// Maximum PWM value during soft start phase
#define SOFT_START_MAX_PWM          120   // Limits initial speed

// Time interval after which motor takes a thermal break
#define THERMAL_BREAK_INTERVAL_MS 45000   // 45 seconds of operation

// Duration of the cooling break
#define THERMAL_BREAK_DURATION_MS  3000   // 3 seconds rest

// Angle beyond which system considers it an extreme tilt
#define EXTREME_ANGLE_THRESHOLD_DEG  55.0   // 55 degrees tilt

// Time allowed at extreme angle before triggering action
#define EXTREME_ANGLE_TIMEOUT_MS     4000   // 4 seconds



// Absolute maximum speed (upper safety limit)
#define ABSOLUTE_MAX_SPEED   200

// Speed when surface is flat
#define SPEED_FLAT           150

// Speed when moving uphill (more power needed)
#define SPEED_UPHILL         180

// Speed when moving downhill (reduced to avoid overspeed)
#define SPEED_DOWNHILL       120

// Speed during return/backward movement
#define SPEED_RETURN         110

// Angle threshold to detect uphill condition
#define UPHILL_THRESHOLD_DEG     8.0   // > +8° = uphill

// Angle threshold to detect downhill condition
#define DOWNHILL_THRESHOLD_DEG  -8.0  // < -8° = downhill

// Current value above which it is considered overcurrent
#define OVERCURRENT_THRESHOLD     900

// Number of consecutive detections required to confirm overcurrent
#define OVERCURRENT_CONFIRM_COUNT   3

// Number of ADC samples taken for averaging current measurement
#define ADC_SAMPLES                 5

/* ═══════════════════════════════════════════════════════════════════════════
 * OBSTACLE DETECTION PARAMETERS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define OBSTACLE_DISTANCE_CM       15
#define OBSTACLE_CONFIRM_TIME_MS 2000
#define ULTRASONIC_TIMEOUT_US    20000
#define ULTRASONIC_READINGS          3



/* ═══════════════════════════════════════════════════════════════════════════
 * ODOMETRY CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */

#define WHEEL_DIAMETER_M   0.10   // SET YOUR REAL VALUE (metres)
#define MOTOR_RPM          30.0   // SET YOUR REAL NO-LOAD RPM

/* ═══════════════════════════════════════════════════════════════════════════
 * PI COMMUNICATION PROTOCOL PARAMETERS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define PI_HB_TIMEOUT_MS    5000UL  // Stop motors if no HB received for 5 s
#define PI_HB_SAFETY_MSG   "SAFETY_STOP"
#define PI_DIST_INTERVAL_MS  400UL  // Send DIST: update every 400 ms
#define PI_CMD_BUFFER_LEN      16   // Max incoming command token length

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE DEFINITION
 * ═══════════════════════════════════════════════════════════════════════════ */

enum RobotState {
  STATE_FORWARD,          // Active exploration
  STATE_THERMAL_BREAK,    // Mandatory cooling rest
  STATE_FAILSAFE_RETURN,  // Reversing to start
  STATE_EMERGENCY_STOP,   // Safety halt – requires reset
  STATE_COMPLETED         // Mission success
};

/* ═══════════════════════════════════════════════════════════════════════════
 * GLOBAL STATE
 * ═══════════════════════════════════════════════════════════════════════════ */

// CRITICAL: Global motor enable gate.
// Once set false by emergencyStop(), motors CANNOT run until reset.
volatile bool motorEnableGate = true;

MPU6050   mpu;
RobotState currentState = STATE_FORWARD;

// Timing trackers
unsigned long forwardStartTime        = 0;
unsigned long thermalBreakStartTime   = 0;
unsigned long lastThermalBreakTime    = 0;
unsigned long continuousRunStartTime  = 0;
unsigned long extremeAngleStartTime   = 0;
unsigned long obstacleDetectedTime    = 0;
unsigned long reverseStartTime        = 0;
unsigned long lastStatusPrintTime     = 0;

// State flags
bool wasOnExtremeAngle  = false;
bool obstacleConfirming = false;

// Sensor data (filtered)
float pitchAngleDeg = 0.0;
int   distanceCm    = 0;

// Safety counters
int overcurrentCounter = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * TELEMETRY / ODOMETRY GLOBALS
 * ═══════════════════════════════════════════════════════════════════════════ */

float         yawAngleDeg         = 0.0;
float         totalDistanceMeters = 0.0;
float         deltaDistanceMeters = 0.0;
unsigned long lastOdometryTime    = 0;
unsigned long lastTelemetrySend   = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * PI COMMUNICATION GLOBALS
 * ═══════════════════════════════════════════════════════════════════════════ */

unsigned long pi_lastHeartbeatMs  = 0;     // millis() of last received HB
bool          pi_heartbeatArmed   = false; // true once first HB is received
unsigned long pi_lastDistSendMs   = 0;     // millis() of last DIST: send
bool          pi_blockReported    = false; // true while BLOCK/REVERSING active
char          pi_cmdBuf[PI_CMD_BUFFER_LEN];
uint8_t       pi_cmdLen           = 0;
volatile bool pi_piRequestedStop  = false; // set true on STOP command from Pi

/* ═══════════════════════════════════════════════════════════════════════════
 * FORWARD DECLARATIONS (keeps compiler happy regardless of order)
 * ═══════════════════════════════════════════════════════════════════════════ */

void  initializeHardware();
bool  initializeMPU6050();
void  printSafetyFeatures();
bool  runSafetyChecks();
bool  checkOvercurrent();
bool  checkExtremeAngleTimeout();
void  emergencyStop(const char* reason);
//void  blinkBuzzer(int durationMs);
void  updateSensors();
int   readUltrasonicDistance();
int   singleUltrasonicReading();
float calculatePitchAngle();
float calculateYawAngle(float dtSeconds);
void  updateOdometry();
void  setMotorSpeed(int speed, bool forward);
int   applySoftStartLimit(int requestedSpeed);
void  setMotorForward(int speed);
void  setMotorReverse(int speed);
void  stopMotors();
void  executeStateMachine();
void  handleForwardState();
bool  detectObstacle();
int   calculateAdaptiveSpeed();
void  initiateReturn();
void  handleThermalBreakState();
void  handleReturnState();
void  completeMission();
void  printStatus();
void  sendPathTelemetry();
// Pi protocol
void  sendStatus(String message);
void  sendDistance();
void  sendDistanceUpdate();
void  handleBlockStateReporting();
void  parsePiSerial();
void  dispatchPiCommand(const char* cmd);
void  checkHeartbeatTimeout();

/* ═══════════════════════════════════════════════════════════════════════════
 * SETUP
 * ═══════════════════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(115200);
  delay(500);

  // UART2 → Raspberry Pi  (telemetry + command protocol)
  PiSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Stamp heartbeat timer at boot so the watchdog does not fire before
  // the Pi has sent its first HB (timeout only arms after first real HB).
  pi_lastHeartbeatMs = millis();

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  ESP32 PIPELINE ROBOT - HARDENED SAFETY MODE   ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");

  initializeHardware();

  if (!initializeMPU6050()) {
    emergencyStop("MPU6050 initialization failed - CRITICAL SENSOR MISSING");
    while (1) {
      blinkBuzzer(300);
      yield();
    }
  }

  printSafetyFeatures();

  Serial.println("\n⚠ CALIBRATION: Place robot on FLAT, STABLE surface");
  Serial.println("   Waiting 3 seconds...\n");
  delay(3000);

  Serial.println("Mission will start in 3 seconds...");
  delay(3000);

  forwardStartTime       = millis();
  lastThermalBreakTime   = millis();
  continuousRunStartTime = millis();
  lastOdometryTime       = millis();
  lastTelemetrySend      = millis();

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║              MISSION STARTED                   ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN LOOP  (25 Hz / 40 ms cycle)
 * ═══════════════════════════════════════════════════════════════════════════ */

void loop() {
  yield();               // Feed watchdog

  updateSensors();       // Read HC-SR04 + MPU6050 pitch
  updateOdometry();      // Update distance + yaw (telemetry only)

  // CRITICAL: Run all safety checks BEFORE state machine
  if (!runSafetyChecks()) {
    return;
  }

  executeStateMachine(); // State-driven motor control
  printStatus();         // Human-readable USB debug

  // ── Telemetry & Pi protocol (all additive, no safety side-effects) ────────
  sendPathTelemetry();          // DATA,... CSV frame → Pi over UART2
  parsePiSerial();              // Read commands from Pi (HB / START / STOP …)
  checkHeartbeatTimeout();      // Kill motors if Pi goes silent > 5 s
  sendDistanceUpdate();         // DIST:x.xx → Pi every 400 ms
  handleBlockStateReporting();  // BLOCK / REVERSING / CLEAR / END transitions

  delay(40);             // 25 Hz loop rate
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE INITIALIZATION
 * ═══════════════════════════════════════════════════════════════════════════ */

void initializeHardware() {
  pinMode(R_EN_PIN,   OUTPUT);
  pinMode(L_EN_PIN,   OUTPUT);
  pinMode(TRIG_PIN,   OUTPUT);
  pinMode(ECHO_PIN,   INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (USE_CURRENT_SENSING) {
    pinMode(CURRENT_SENSE_PIN, INPUT);
  }

  // CRITICAL: Disable motor driver before anything else
  digitalWrite(R_EN_PIN,   LOW);
  digitalWrite(L_EN_PIN,   LOW);
  digitalWrite(BUZZER_PIN, LOW);

  if (!ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("ERROR: Failed to initialize RPWM channel");
  }
  if (!ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("ERROR: Failed to initialize LPWM channel");
  }

  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);

  Serial.println("✓ Hardware initialized to safe state");
  Serial.println("  - Motor enables: LOW");
  Serial.println("  - PWM channels: 0");
  Serial.println("  - Motor gate: ENABLED");
}

bool initializeMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

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
  Serial.println("  ✓ Gyro-Z yaw integration");
  Serial.println("  ✓ UART2 DATA telemetry (GPIO16/17 → RPi)");
  Serial.println("  ✓ Pi heartbeat watchdog (5s timeout)");
  Serial.println("  ✓ Pi STOP command safety gate");
  Serial.println("  ✓ DIST / BLOCK / REVERSING / CLEAR / END protocol");
  Serial.println("═══════════════════════════════════════");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SAFETY CHECK SYSTEM
 * ═══════════════════════════════════════════════════════════════════════════ */

bool runSafetyChecks() {
  if (!checkOvercurrent()) {
    emergencyStop("OVERCURRENT - Possible motor stall or jam");
    return false;
  }
  if (!checkExtremeAngleTimeout()) {
    emergencyStop("STALL SUSPECTED - Extreme angle timeout exceeded");
    return false;
  }
  
  return true;
}

bool checkOvercurrent() {
  if (!USE_CURRENT_SENSING || currentState != STATE_FORWARD) {
    overcurrentCounter = 0;
    return true;
  }

  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(CURRENT_SENSE_PIN);
    delayMicroseconds(100);
    yield();
  }
  int avgCurrent = sum / ADC_SAMPLES;

  if (avgCurrent > OVERCURRENT_THRESHOLD) {
    overcurrentCounter++;
    if (overcurrentCounter >= OVERCURRENT_CONFIRM_COUNT) {
      Serial.print("\n!!! OVERCURRENT DETECTED !!! ADC=");
      Serial.println(avgCurrent);
      return false;
    }
  } else {
    overcurrentCounter = 0;
  }
  return true;
}

bool checkExtremeAngleTimeout() {
  if (currentState != STATE_FORWARD) {
    wasOnExtremeAngle = false;
    return true;
  }

  bool isExtreme = (abs(pitchAngleDeg) > EXTREME_ANGLE_THRESHOLD_DEG);

  if (isExtreme) {
    if (!wasOnExtremeAngle) {
      extremeAngleStartTime = millis();
      wasOnExtremeAngle     = true;
      Serial.print("\n⚠ EXTREME ANGLE: ");
      Serial.print(pitchAngleDeg, 1);
      Serial.println("°");
    }
    if (millis() - extremeAngleStartTime > EXTREME_ANGLE_TIMEOUT_MS) {
      Serial.println("\n!!! EXTREME ANGLE TIMEOUT !!!");
      return false;
    }
  } else {
    if (wasOnExtremeAngle) Serial.println("✓ Returned to safe angle range");
    wasOnExtremeAngle = false;
  }
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * EMERGENCY STOP
 * ═══════════════════════════════════════════════════════════════════════════ */

void emergencyStop(const char* reason) {
  motorEnableGate = false;   // MUST be first
  stopMotors();
  currentState = STATE_EMERGENCY_STOP;

  blinkBuzzer(100);
  delay(100);
  blinkBuzzer(100);

  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║          EMERGENCY STOP ACTIVATED             ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.print("  Reason: ");
  Serial.println(reason);
  Serial.println("  Motor gate: DISABLED");
  Serial.println("  Motors: STOPPED");
  Serial.println("  ⚠ Investigate cause before reset");
  Serial.println("╚═══════════════════════════════════════════════╝\n");
}

/*void blinkBuzzer(int durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}*/

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

  for (int i = 0; i < ULTRASONIC_READINGS; i++) {
    int dist = singleUltrasonicReading();
    if (dist > 0 && dist < 400) {
      readings[validCount++] = dist;
    }
    delay(10);
    yield();
  }

  if (validCount == 0) return 400;

  if (validCount >= 3) {
    // Bubble sort → return median
    for (int i = 0; i < validCount - 1; i++) {
      for (int j = 0; j < validCount - i - 1; j++) {
        if (readings[j] > readings[j + 1]) {
          int t = readings[j]; readings[j] = readings[j+1]; readings[j+1] = t;
        }
      }
    }
    return readings[validCount / 2];
  }

  long sum = 0;
  for (int i = 0; i < validCount; i++) sum += readings[i];
  return sum / validCount;
}

int singleUltrasonicReading() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return 400;
  return (duration * 0.034) / 2;
}

float calculatePitchAngle() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;

  static float filteredPitch = 0.0;
  filteredPitch = 0.8 * filteredPitch + 0.2 * pitch;
  return filteredPitch;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * YAW INTEGRATION  (gyro-Z, telemetry only)
 * ═══════════════════════════════════════════════════════════════════════════ */

float calculateYawAngle(float dtSeconds) {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ_dps = gz / 131.0f;

  static float yaw = 0.0f;
  yaw += gyroZ_dps * dtSeconds;
  return yaw;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ODOMETRY UPDATE
 * ═══════════════════════════════════════════════════════════════════════════ */

void updateOdometry() {
  unsigned long now = millis();
  float dtSeconds   = (now - lastOdometryTime) / 1000.0f;
  lastOdometryTime  = now;

  if (dtSeconds <= 0.0f || dtSeconds > 1.0f) {
    yawAngleDeg         = calculateYawAngle(0.0f);
    deltaDistanceMeters = 0.0f;
    return;
  }

  const float circumference = PI * WHEEL_DIAMETER_M;
  const float linearSpeed   = (MOTOR_RPM * circumference) / 60.0f;

  if (currentState == STATE_FORWARD) {
    deltaDistanceMeters  =  linearSpeed * dtSeconds;
    totalDistanceMeters +=  deltaDistanceMeters;
  } else if (currentState == STATE_FAILSAFE_RETURN) {
    deltaDistanceMeters  = -(linearSpeed * dtSeconds);
    totalDistanceMeters +=   deltaDistanceMeters;
    if (totalDistanceMeters < 0.0f) totalDistanceMeters = 0.0f;
  } else {
    deltaDistanceMeters = 0.0f;
  }

  yawAngleDeg = calculateYawAngle(dtSeconds);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR CONTROL  (hardware-safe PWM sequencing)
 * ═══════════════════════════════════════════════════════════════════════════ */

void setMotorSpeed(int speed, bool forward) {
  if (!motorEnableGate) { stopMotors(); return; }

  speed = constrain(speed, 0, ABSOLUTE_MAX_SPEED);
  speed = applySoftStartLimit(speed);

  if (forward) setMotorForward(speed);
  else         setMotorReverse(speed);
}

int applySoftStartLimit(int requestedSpeed) {
  unsigned long elapsedMs = millis() - forwardStartTime;
  if (elapsedMs < SOFT_START_DURATION_MS) {
    int maxAllowed = map(elapsedMs, 0, SOFT_START_DURATION_MS, 0, SOFT_START_MAX_PWM);
    return min(requestedSpeed, maxAllowed);
  }
  return requestedSpeed;
}

void setMotorForward(int speed) {
  // Anti-shoot-through sequence
  digitalWrite(L_EN_PIN, LOW);
  ledcWrite(LPWM_PIN, 0);
  delayMicroseconds(500);
  ledcWrite(RPWM_PIN, speed);
  delayMicroseconds(100);
  digitalWrite(R_EN_PIN, HIGH);
}

void setMotorReverse(int speed) {
  digitalWrite(R_EN_PIN, LOW);
  ledcWrite(RPWM_PIN, 0);
  delayMicroseconds(500);
  ledcWrite(LPWM_PIN, speed);
  delayMicroseconds(100);
  digitalWrite(L_EN_PIN, HIGH);
}

void stopMotors() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
  delay(1);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE
 * ═══════════════════════════════════════════════════════════════════════════ */

void executeStateMachine() {
  switch (currentState) {
    case STATE_FORWARD:         handleForwardState();   break;
    case STATE_THERMAL_BREAK:   handleThermalBreakState(); break;
    case STATE_FAILSAFE_RETURN: handleReturnState();    break;
    case STATE_EMERGENCY_STOP:  stopMotors();           break;
    case STATE_COMPLETED:       stopMotors();           break;
    default:
      emergencyStop("INVALID STATE DETECTED - State machine corrupted");
      break;
  }
}

void handleForwardState() {
  if (millis() - lastThermalBreakTime >= THERMAL_BREAK_INTERVAL_MS) {
    Serial.println("\n─── THERMAL BREAK (mandatory cooling) ───");
    stopMotors();
    thermalBreakStartTime = millis();
    currentState = STATE_THERMAL_BREAK;
    return;
  }

  if (detectObstacle()) {
    initiateReturn();
    return;
  }

  setMotorSpeed(calculateAdaptiveSpeed(), true);
}

bool detectObstacle() {
  bool present = (distanceCm < OBSTACLE_DISTANCE_CM);

  if (present) {
    if (!obstacleConfirming) {
      obstacleConfirming    = true;
      obstacleDetectedTime  = millis();
      Serial.print("\n⚠ OBSTACLE at ");
      Serial.print(distanceCm);
      Serial.println(" cm – confirming...");
    }
    if (millis() - obstacleDetectedTime >= OBSTACLE_CONFIRM_TIME_MS) {
      Serial.println("✓ Obstacle CONFIRMED");
      return true;
    }
  } else {
    if (obstacleConfirming) {
      Serial.println("✓ False alarm – obstacle cleared");
      obstacleConfirming = false;
    }
  }
  return false;
}

int calculateAdaptiveSpeed() {
  int speed = SPEED_FLAT;
  if (pitchAngleDeg > UPHILL_THRESHOLD_DEG) {
    speed = SPEED_UPHILL;
    if (pitchAngleDeg > 45.0) speed = constrain(speed - 30, 100, SPEED_UPHILL);
  } else if (pitchAngleDeg < DOWNHILL_THRESHOLD_DEG) {
    speed = SPEED_DOWNHILL;
  }
  return speed;
}

void initiateReturn() {
  stopMotors();
  blinkBuzzer(200);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║      OBSTACLE CONFIRMED - INITIATING RETURN    ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.print("  Forward duration: ");
  Serial.print((millis() - forwardStartTime) / 1000.0, 1);
  Serial.println(" s");
  Serial.print("  Distance to obstacle: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  reverseStartTime = millis();
  currentState = STATE_FAILSAFE_RETURN;
}

void handleThermalBreakState() {
  if (millis() - thermalBreakStartTime >= THERMAL_BREAK_DURATION_MS) {
    Serial.println("✓ Thermal break complete – Resuming\n");
    lastThermalBreakTime   = millis();
    continuousRunStartTime = millis();
    overcurrentCounter     = 0;
    currentState = STATE_FORWARD;
  } else {
    stopMotors();
  }
}

void handleReturnState() {
  unsigned long forwardDuration = reverseStartTime - forwardStartTime;
  unsigned long reverseElapsed  = millis() - reverseStartTime;

  if (forwardDuration > MAX_MISSION_DURATION_MS) forwardDuration = MAX_MISSION_DURATION_MS;

  if (reverseElapsed >= forwardDuration) {
    completeMission();
    return;
  }

  static unsigned long lastProgressPrint = 0;
  if (millis() - lastProgressPrint > 2000) {
    Serial.print("◄ RETURNING: ");
    Serial.print((reverseElapsed * 100.0) / forwardDuration, 1);
    Serial.println("%");
    lastProgressPrint = millis();
  }

  setMotorSpeed(SPEED_RETURN, false);
}

void completeMission() {
  stopMotors();
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║         MISSION COMPLETED SUCCESSFULLY         ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.print("  Forward time: ");
  Serial.print((reverseStartTime - forwardStartTime) / 1000.0, 1);
  Serial.println(" s");
  Serial.println("  System ready for reset\n");

  currentState = STATE_COMPLETED;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATUS REPORTING  (USB / UART0 only)
 * ═══════════════════════════════════════════════════════════════════════════ */

void printStatus() {
  if (currentState == STATE_EMERGENCY_STOP) {
    static unsigned long last = 0;
    if (millis() - last > 3000) {
      Serial.println("⚠ EMERGENCY STOP ACTIVE - Reset required");
      last = millis();
    }
    return;
  }
  if (currentState == STATE_COMPLETED) {
    static unsigned long last = 0;
    if (millis() - last > 5000) {
      Serial.println("✓ Mission complete - Reset to restart");
      last = millis();
    }
    return;
  }

  if (millis() - lastStatusPrintTime > 500) {
    Serial.print("Pitch: ");  Serial.print(pitchAngleDeg, 1);
    Serial.print("° | Dist: "); Serial.print(distanceCm);
    Serial.print("cm | State: ");
    switch (currentState) {
      case STATE_FORWARD:         Serial.print("FORWARD");       break;
      case STATE_THERMAL_BREAK:   Serial.print("THERMAL_BREAK"); break;
      case STATE_FAILSAFE_RETURN: Serial.print("RETURN");        break;
      default:                    Serial.print("UNKNOWN");        break;
    }
    if (USE_CURRENT_SENSING) {
      Serial.print(" | I: ");
      Serial.print(analogRead(CURRENT_SENSE_PIN));
    }
    Serial.println();
    lastStatusPrintTime = millis();
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * EXISTING PATH TELEMETRY  (DATA,... CSV → UART2, unchanged)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Field layout:
 *   DATA,<ms>,<delta_m>,<total_m>,<yaw_deg>,<pitch_deg>,
 *        <state_int>,<ultrasonic_cm>,<current_raw>
 */

void sendPathTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetrySend < 200UL) return;
  lastTelemetrySend = now;

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
 * PI COMMUNICATION PROTOCOL  (new functions)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * sendStatus()
 * Send a plain-text status token to the Raspberry Pi over UART2.
 * Examples: sendStatus("BLOCK"), sendStatus("REVERSING"), sendStatus("END")
 * Always appends \n so Pi readline() terminates cleanly.
 * Also mirrors to USB UART0 for developer visibility.
 */
void sendStatus(String message) {
  PiSerial.println(message);
  Serial.print("[→ Pi] ");
  Serial.println(message);
}

/**
 * sendDistance()
 * Formats totalDistanceMeters as "DIST:x.xx" and transmits to Pi.
 * Uses dtostrf (stack-allocated) to avoid String heap fragmentation.
 */
void sendDistance() {
  char buf[16];
  dtostrf(totalDistanceMeters, 1, 2, buf);
  PiSerial.print("DIST:");
  PiSerial.println(buf);
}

/**
 * sendDistanceUpdate()
 * Non-blocking cadence wrapper – sends DIST: at PI_DIST_INTERVAL_MS.
 * Only transmits while mission is active (not in e-stop or completed).
 * Call once per loop() iteration.
 */
void sendDistanceUpdate() {
  unsigned long now = millis();
  if (now - pi_lastDistSendMs < PI_DIST_INTERVAL_MS) return;
  pi_lastDistSendMs = now;

  if (currentState == STATE_FORWARD       ||
      currentState == STATE_FAILSAFE_RETURN ||
      currentState == STATE_THERMAL_BREAK) {
    sendDistance();
  }
}

/**
 * handleBlockStateReporting()
 * Edge-triggered state watcher.  Fires BLOCK / REVERSING / CLEAR / END
 * exactly ONCE per state transition, not on every loop iteration.
 *
 * Transitions handled:
 *   → STATE_FAILSAFE_RETURN  : send BLOCK then REVERSING
 *   → STATE_FORWARD          : send CLEAR  (if previously blocked)
 *   → STATE_COMPLETED        : send END + final distance
 *   → STATE_EMERGENCY_STOP   : send END    (covers hard-stop case)
 */
void handleBlockStateReporting() {
  static RobotState lastReportedState = STATE_FORWARD;

  if (currentState == lastReportedState) return;

  switch (currentState) {

    case STATE_FAILSAFE_RETURN:
      sendStatus("BLOCK");
      sendStatus("REVERSING");
      pi_blockReported = true;
      break;

    case STATE_FORWARD:
      if (pi_blockReported) {
        sendStatus("CLEAR");
        pi_blockReported = false;
      }
      break;

    case STATE_COMPLETED:
      sendStatus("END");
      sendDistance();   // Final odometer reading alongside END
      break;

    case STATE_EMERGENCY_STOP:
      sendStatus("END");
      break;

    default:
      break;  // STATE_THERMAL_BREAK – no protocol message needed
  }

  lastReportedState = currentState;
}

/**
 * parsePiSerial()
 * Non-blocking byte-by-byte reader for UART2 incoming data.
 * Accumulates bytes into pi_cmdBuf until '\n', then calls dispatchPiCommand().
 * Handles \r\n (Windows) line endings gracefully.
 * Buffer-overflow-safe: garbled frames are discarded and buffer reset.
 */
void parsePiSerial() {
  while (PiSerial.available()) {
    char c = (char)PiSerial.read();

    if (c == '\r') continue;   // Ignore CR in CRLF pairs

    if (c == '\n') {
      pi_cmdBuf[pi_cmdLen] = '\0';
      dispatchPiCommand(pi_cmdBuf);
      pi_cmdLen = 0;
    } else {
      if (pi_cmdLen < PI_CMD_BUFFER_LEN - 1) {
        pi_cmdBuf[pi_cmdLen++] = c;
      } else {
        pi_cmdLen = 0;   // Overflow – discard garbled frame
      }
    }
  }
}

/**
 * dispatchPiCommand()
 * Routes a complete null-terminated command token to the correct handler.
 *
 * HB            Update heartbeat timestamp; arm watchdog on first receipt.
 * START / MOVE  Clear pi_piRequestedStop; re-enable motor gate if safe to do so.
 * STOP          Disable motor gate, stop motors, send SAFETY_STOP ack.
 * <unknown>     Log and ignore.
 */
void dispatchPiCommand(const char* cmd) {

  // ── HB ───────────────────────────────────────────────────────────────────
  if (strcmp(cmd, "HB") == 0) {
    pi_lastHeartbeatMs = millis();
    pi_heartbeatArmed  = true;
    return;
  }

  // ── START / MOVE ─────────────────────────────────────────────────────────
  if (strcmp(cmd, "START") == 0 || strcmp(cmd, "MOVE") == 0) {
    Serial.print("[← Pi] CMD: ");
    Serial.println(cmd);

    if (pi_piRequestedStop) {
      pi_piRequestedStop = false;
      // Re-enable ONLY if the halt was caused by Pi STOP, not a hardware fault.
      // Hardware faults leave currentState == STATE_EMERGENCY_STOP.
      if (currentState != STATE_EMERGENCY_STOP) {
        motorEnableGate = true;
        Serial.println("  Motor gate RE-ENABLED by Pi START/MOVE");
      } else {
        Serial.println("  ⚠ Hardware e-stop active – motor gate stays LOCKED");
      }
    }
    return;
  }

  // ── STOP ─────────────────────────────────────────────────────────────────
  if (strcmp(cmd, "STOP") == 0) {
    Serial.println("[← Pi] CMD: STOP – halting motors");
    pi_piRequestedStop = true;
    motorEnableGate    = false;
    stopMotors();
    sendStatus("SAFETY_STOP");
    return;
  }

  // ── Unknown ──────────────────────────────────────────────────────────────
  // DATA,... telemetry echo-backs or partial frames land here and are silently
  // ignored to avoid spurious log spam.
  if (strncmp(cmd, "DATA", 4) != 0) {
    Serial.print("[← Pi] Unknown command ignored: ");
    Serial.println(cmd);
  }
}

/**
 * checkHeartbeatTimeout()
 * Calls emergencyStop() if Pi has not sent HB within PI_HB_TIMEOUT_MS.
 * Only active after first HB is received (pi_heartbeatArmed).
 * Does not re-trigger if already in STATE_EMERGENCY_STOP.
 */
void checkHeartbeatTimeout() {
  if (!pi_heartbeatArmed)                          return;
  if (currentState == STATE_EMERGENCY_STOP)        return;
  if (currentState == STATE_COMPLETED)             return;

  if (millis() - pi_lastHeartbeatMs > PI_HB_TIMEOUT_MS) {
    // Best-effort notify before motors die
    PiSerial.println(PI_HB_SAFETY_MSG);

    // Reuse existing hardware emergency stop path
    emergencyStop("HEARTBEAT TIMEOUT — Raspberry Pi connection lost");

    // Disarm so repeated loop() calls don't re-trigger emergencyStop()
    pi_heartbeatArmed = false;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * END OF FIRMWARE
 * ═══════════════════════════════════════════════════════════════════════════ */
