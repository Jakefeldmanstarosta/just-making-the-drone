// =============================================================================
//  ESP32 Quadcopter Flight Controller
//  Target:   Custom 4S / 920kv / 1045-prop Quad-X
//  MCU:      ESP32
//  IMU:      MPU6050 (I2C: SDA=GPIO22, SCL=GPIO21)
//  ESCs:     30A SimonK, Standard PWM 1000-2000µs
//            FL=GPIO12  FR=GPIO13  BL=GPIO14  BR=GPIO26
//  Libs:     ESP32Servo, Wire (built-in)
// =============================================================================

#include <Wire.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define PIN_ESC_FL   12   // Front-Left  motor
#define PIN_ESC_FR   13   // Front-Right motor
#define PIN_ESC_BL   14   // Back-Left   motor
#define PIN_ESC_BR   26   // Back-Right  motor

#define PIN_VBAT     34   // Analog input for battery voltage divider
                          // (GPIO34 is input-only – safe choice on ESP32)

// ---------------------------------------------------------------------------
// I2C / MPU6050 Constants
// ---------------------------------------------------------------------------
#define MPU6050_ADDR      0x68
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_SMPLRT    0x19
#define MPU6050_CONFIG    0x1A
#define MPU6050_GYRO_CFG  0x1B
#define MPU6050_ACCEL_CFG 0x1C
#define MPU6050_ACCEL_OUT 0x3B   // First byte of 14-byte burst (ACCEL + TEMP + GYRO)

// Scale factors for ±2g / ±250°/s defaults
#define ACCEL_SCALE  16384.0f   // LSB/g
#define GYRO_SCALE    131.0f   // LSB/(°/s)

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define LOOP_TIME_US  4000          // 250 Hz  (4 000 µs)
#define CALIB_SAMPLES  2000         // gyro calibration iterations (~8 s at 250Hz)

// ---------------------------------------------------------------------------
// Complementary Filter Coefficient
// ---------------------------------------------------------------------------
#define CF_ALPHA   0.98f            // trust gyro heavily; 0.02 accel correction

// ---------------------------------------------------------------------------
// PID Tuning  –  START CONSERVATIVE for 4S high-thrust platform
// ---------------------------------------------------------------------------
// !! IMPORTANT: Tune on the bench with props OFF first. !!
// Typical 920kv 4S starting point:
//   Kp ~1.0-2.0,  Ki ~0.02-0.05,  Kd ~5-15
//
float KP_ROLL   = 1.0f;
float KI_ROLL   = 0.03f;
float KD_ROLL   = 5.0f;

float KP_PITCH  = 1.0f;
float KI_PITCH  = 0.03f;
float KD_PITCH  = 5.0f;

// Anti-windup: clamp integral accumulator (in µs equivalent)
#define INTEGRAL_LIMIT  200.0f

// ---------------------------------------------------------------------------
// Motor Mixing Parameters
// ---------------------------------------------------------------------------
// Base throttle at 1100µs for the aggressive 4S power curve.
// Raise carefully during hover tuning.
#define BASE_THROTTLE   1100
#define THROTTLE_MIN    1000
#define THROTTLE_MAX    2000
#define MOTOR_IDLE       950   // below this ESCs are disarmed / beep

// ---------------------------------------------------------------------------
// Safety
// ---------------------------------------------------------------------------
#define TILT_KILLSWITCH_DEG  45.0f   // ± degrees to trigger kill
#define VBAT_WARN_VOLTS       14.0f
#define VBAT_DIVIDER_RATIO    11.0f  // e.g. 10kΩ + 1kΩ divider → ratio ~11
                                      // adjust for your actual resistor pair
#define VBAT_ATTEN_COEFF      (3.3f / 4095.0f)  // 12-bit ADC, 3.3V ref

// ---------------------------------------------------------------------------
// Servo / ESC Objects
// ---------------------------------------------------------------------------
Servo escFL, escFR, escBL, escBR;

// ---------------------------------------------------------------------------
// IMU State
// ---------------------------------------------------------------------------
struct RawIMU {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Complementary filter angles (degrees)
float anglePitch = 0.0f;
float angleRoll  = 0.0f;

// ---------------------------------------------------------------------------
// PID State
// ---------------------------------------------------------------------------
float pidRollPrev   = 0.0f, pidRollIntegral   = 0.0f;
float pidPitchPrev  = 0.0f, pidPitchIntegral  = 0.0f;

// ---------------------------------------------------------------------------
// Setpoints  –  0° hover target  (replace with RC receiver input later)
// ---------------------------------------------------------------------------
float setpointRoll  = 0.0f;
float setpointPitch = 0.0f;

// ---------------------------------------------------------------------------
// Timing bookkeeping
// ---------------------------------------------------------------------------
uint32_t loopTimer = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void     initMPU6050();
void     calibrateGyro();
bool     readIMU(RawIMU &raw);
void     updateAngles(const RawIMU &raw, float dt);
float    computePID(float setpoint, float measurement,
                    float kp, float ki, float kd,
                    float &integral, float &prevError, float dt);
void     writeMotors(int fl, int fr, int bl, int br);
void     killMotors();
float    readBatteryVoltage();
void     checkBattery();
void     i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  i2cReadByte(uint8_t addr, uint8_t reg);

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n=== ESP32 Quad Flight Controller ==="));

    // ---- I2C ----------------------------------------------------------------
    Wire.begin(22, 21);          // SDA=22, SCL=21
    Wire.setClock(400000);       // 400 kHz fast-mode

    // ---- ESC Initialisation -------------------------------------------------
    // ESP32Servo: attempt 400Hz refresh; library falls back to 50Hz if unsupported.
    // writeMicroseconds() accuracy is unaffected either way.
    escFL.setPeriodHertz(400);
    escFR.setPeriodHertz(400);
    escBL.setPeriodHertz(400);
    escBR.setPeriodHertz(400);

    escFL.attach(PIN_ESC_FL, THROTTLE_MIN, THROTTLE_MAX);
    escFR.attach(PIN_ESC_FR, THROTTLE_MIN, THROTTLE_MAX);
    escBL.attach(PIN_ESC_BL, THROTTLE_MIN, THROTTLE_MAX);
    escBR.attach(PIN_ESC_BR, THROTTLE_MIN, THROTTLE_MAX);

    // Send disarm signal immediately so ESCs don't time-out
    killMotors();
    Serial.println(F("[ESC ] All ESCs set to 1000µs (DISARMED)"));

    // ---- MPU6050 Init -------------------------------------------------------
    initMPU6050();
    Serial.println(F("[IMU ] MPU6050 initialised"));

    // ---- Arming Sequence: hold 1000µs for 5 s while calibrating gyro --------
    Serial.println(F("[ARM ] Holding disarm for 5s – gyro calibration in progress..."));
    Serial.println(F("[ARM ] *** Keep drone perfectly STILL on a flat surface ***"));

    calibrateGyro();   // ~5-8 seconds, prints progress
    Serial.println(F("[ARM ] Gyro calibration complete. ARMING NOW."));
    Serial.printf(  "[ARM ] Offsets  GX:%.2f  GY:%.2f  GZ:%.2f\n",
                    gyroOffsetX, gyroOffsetY, gyroOffsetZ);

    // ---- Analog (battery) pin -----------------------------------------------
    analogReadResolution(12);    // ESP32 default, but be explicit
    analogSetAttenuation(ADC_11db);  // 0-3.3V range

    Serial.println(F("[SYS ] Entering main loop at 250 Hz."));
    Serial.println(F("---------------------------------------"));

    loopTimer = micros();
}

// =============================================================================
//  MAIN LOOP  –  250 Hz strict timing
// =============================================================================
void loop() {
    // ---- Wait for next 4000 µs slot -----------------------------------------
    while (micros() - loopTimer < LOOP_TIME_US) { /* tight spin – no delay() */ }
    float dt = (float)(micros() - loopTimer) / 1000000.0f;   // actual dt in seconds
    loopTimer = micros();

    // ---- Read IMU -----------------------------------------------------------
    RawIMU raw;
    if (!readIMU(raw)) {
        // I2C read failure: hold last motor values, try next cycle
        return;
    }

    // ---- Complementary Filter -----------------------------------------------
    updateAngles(raw, dt);

    // ---- Safety Tilt Kill-Switch  -------------------------------------------
    if (fabsf(anglePitch) > TILT_KILLSWITCH_DEG ||
        fabsf(angleRoll)  > TILT_KILLSWITCH_DEG) {
        killMotors();
        Serial.printf("[KILL] TILT EXCEEDED: Pitch=%.1f° Roll=%.1f° – MOTORS OFF\n",
                      anglePitch, angleRoll);
        // Spin here; require power-cycle to re-arm (safety measure)
        while (true) { delay(1000); }
    }

    // ---- Battery Check (non-blocking, every cycle) --------------------------
    checkBattery();

    // ---- PID Computation ----------------------------------------------------
    float pidRoll  = computePID(setpointRoll,  angleRoll,
                                KP_ROLL,  KI_ROLL,  KD_ROLL,
                                pidRollIntegral,  pidRollPrev,  dt);

    float pidPitch = computePID(setpointPitch, anglePitch,
                                KP_PITCH, KI_PITCH, KD_PITCH,
                                pidPitchIntegral, pidPitchPrev, dt);

    // (Yaw PID omitted – add gyroZ rate controller here when RC is connected)
    float pidYaw = 0.0f;

    // ---- Motor Mixing  –  Standard Quad-X Layout ----------------------------
    //
    //        FRONT
    //   FL (CCW)  FR (CW)
    //       \      /
    //        +----+
    //       /      \
    //   BL (CW)  BR (CCW)
    //
    // Positive Roll  → right side dips  → increase FR, BR; decrease FL, BL
    // Positive Pitch → nose dips        → increase FL, FR; decrease BL, BR
    // Positive Yaw   → nose turns right → increase FL, BR; decrease FR, BL
    //
    int motorFL = BASE_THROTTLE + pidPitch + pidRoll - pidYaw;
    int motorFR = BASE_THROTTLE + pidPitch - pidRoll + pidYaw;
    int motorBL = BASE_THROTTLE - pidPitch + pidRoll + pidYaw;
    int motorBR = BASE_THROTTLE - pidPitch - pidRoll - pidYaw;

    // Clamp outputs to safe range
    motorFL = constrain(motorFL, THROTTLE_MIN, THROTTLE_MAX);
    motorFR = constrain(motorFR, THROTTLE_MIN, THROTTLE_MAX);
    motorBL = constrain(motorBL, THROTTLE_MIN, THROTTLE_MAX);
    motorBR = constrain(motorBR, THROTTLE_MIN, THROTTLE_MAX);

    writeMotors(motorFL, motorFR, motorBL, motorBR);

    // ---- Debug (comment out in flight to save cycle time) -------------------
    // Serial.printf("P:%.2f R:%.2f | FL:%d FR:%d BL:%d BR:%d\n",
    //               anglePitch, angleRoll, motorFL, motorFR, motorBL, motorBR);
}

// =============================================================================
//  IMU FUNCTIONS
// =============================================================================

void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t i2cReadByte(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

void initMPU6050() {
    // Wake device (clear sleep bit)
    i2cWriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT1, 0x01);  // Use PLL with X-gyro ref
    delay(100);

    // Sample rate: 1kHz / (1 + SMPLRT_DIV)
    // DIV=3 → 250Hz  (matches our loop rate)
    i2cWriteByte(MPU6050_ADDR, MPU6050_SMPLRT, 0x03);

    // DLPF_CFG = 3 → 44Hz accel / 42Hz gyro bandwidth
    // Attenuates high-frequency motor vibrations from 10" props
    i2cWriteByte(MPU6050_ADDR, MPU6050_CONFIG, 0x03);

    // Gyro full scale: ±250°/s (0x00)
    i2cWriteByte(MPU6050_ADDR, MPU6050_GYRO_CFG, 0x00);

    // Accel full scale: ±2g (0x00)
    i2cWriteByte(MPU6050_ADDR, MPU6050_ACCEL_CFG, 0x00);
}

// Collect CALIB_SAMPLES gyro readings with drone perfectly still
void calibrateGyro() {
    long sumX = 0, sumY = 0, sumZ = 0;
    int  printed = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        // --- enforce the same 250Hz pacing during calibration ---
        while (micros() - loopTimer < LOOP_TIME_US) {}
        loopTimer = micros();

        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU6050_ACCEL_OUT);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

        // Discard accel & temp (bytes 0-7)
        for (int b = 0; b < 8; b++) Wire.read();

        int16_t gx = (Wire.read() << 8) | Wire.read();
        int16_t gy = (Wire.read() << 8) | Wire.read();
        int16_t gz = (Wire.read() << 8) | Wire.read();

        sumX += gx;
        sumY += gy;
        sumZ += gz;

        // Print progress every 500 samples
        if (i % 500 == 0) {
            Serial.printf("[CAL ] %d / %d samples...\n", i, CALIB_SAMPLES);
        }
    }

    gyroOffsetX = (float)sumX / CALIB_SAMPLES;
    gyroOffsetY = (float)sumY / CALIB_SAMPLES;
    gyroOffsetZ = (float)sumZ / CALIB_SAMPLES;
}

bool readIMU(RawIMU &raw) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t n = Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);
    if (n < 14) return false;

    raw.ax = (Wire.read() << 8) | Wire.read();
    raw.ay = (Wire.read() << 8) | Wire.read();
    raw.az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();   // temperature – discard
    raw.gx = (Wire.read() << 8) | Wire.read();
    raw.gy = (Wire.read() << 8) | Wire.read();
    raw.gz = (Wire.read() << 8) | Wire.read();

    return true;
}

// ---------------------------------------------------------------------------
//  Complementary Filter
//  Angle = 0.98 * (Angle + Gyro * dt) + 0.02 * Accel
// ---------------------------------------------------------------------------
void updateAngles(const RawIMU &raw, float dt) {
    // Subtract gyro offsets and convert to °/s
    float gyroX = ((float)raw.gx - gyroOffsetX) / GYRO_SCALE;   // roll rate
    float gyroY = ((float)raw.gy - gyroOffsetY) / GYRO_SCALE;   // pitch rate
    // gyroZ (yaw rate) not used by angle filter but available for yaw PID

    // Accel angles (degrees) using small-angle-safe atan2
    // Note: axis mapping depends on MPU6050 physical orientation on your frame.
    //       Flip signs / swap axes if drone responds incorrectly.
    float accelRoll  =  atan2f((float)raw.ay, (float)raw.az) * 57.2957795f;
    float accelPitch = -atan2f((float)raw.ax,
                                sqrtf((float)raw.ay*(float)raw.ay +
                                      (float)raw.az*(float)raw.az)) * 57.2957795f;

    // Complementary filter
    angleRoll  = CF_ALPHA * (angleRoll  + gyroX * dt) + (1.0f - CF_ALPHA) * accelRoll;
    anglePitch = CF_ALPHA * (anglePitch + gyroY * dt) + (1.0f - CF_ALPHA) * accelPitch;
}

// =============================================================================
//  PID CONTROLLER
//  Returns a µs correction value to add to / subtract from motor outputs.
// =============================================================================
float computePID(float setpoint, float measurement,
                 float kp, float ki, float kd,
                 float &integral, float &prevError, float dt) {
    float error      = setpoint - measurement;
    float derivative = (error - prevError) / dt;

    // Integral accumulation with anti-windup clamp
    integral += error * dt;
    integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float output = (kp * error) + (ki * integral) + (kd * derivative);

    prevError = error;
    return output;
}

// =============================================================================
//  MOTOR CONTROL
// =============================================================================
void writeMotors(int fl, int fr, int bl, int br) {
    escFL.writeMicroseconds(fl);
    escFR.writeMicroseconds(fr);
    escBL.writeMicroseconds(bl);
    escBR.writeMicroseconds(br);
}

void killMotors() {
    escFL.writeMicroseconds(THROTTLE_MIN);
    escFR.writeMicroseconds(THROTTLE_MIN);
    escBL.writeMicroseconds(THROTTLE_MIN);
    escBR.writeMicroseconds(THROTTLE_MIN);
}

// =============================================================================
//  BATTERY MONITORING
// =============================================================================
float readBatteryVoltage() {
    // Read 12-bit ADC (0-4095), convert to voltage accounting for divider.
    // Assumes a simple resistive divider:
    //   VBAT ── R1(10kΩ) ── ADC_PIN ── R2(1kΩ) ── GND
    //   V_adc = VBAT * (R2 / (R1 + R2))  →  VBAT = V_adc * (R1+R2)/R2
    //
    // ESP32 ADC is noisy; oversample 16× for stability.
    uint32_t adcSum = 0;
    for (int i = 0; i < 16; i++) adcSum += analogRead(PIN_VBAT);
    float adcAvg     = (float)adcSum / 16.0f;
    float adcVoltage = adcAvg * VBAT_ATTEN_COEFF;
    return adcVoltage * VBAT_DIVIDER_RATIO;
}

// Called every loop – use a counter to avoid Serial spam
void checkBattery() {
    static uint32_t lastCheck = 0;
    static bool     warned    = false;

    // Check every ~1 second (250 loops)
    static uint8_t counter = 0;
    if (++counter < 250) return;
    counter = 0;

    float vbat = readBatteryVoltage();

    if (vbat < VBAT_WARN_VOLTS) {
        if (!warned) {
            Serial.printf("[BATT] WARNING: %.2fV < %.1fV – LAND NOW!\n",
                          vbat, VBAT_WARN_VOLTS);
            warned = true;
        }
    } else {
        warned = false;   // reset if voltage recovers (e.g. you had a load spike)
        // Uncomment for periodic telemetry:
        // Serial.printf("[BATT] %.2fV OK\n", vbat);
    }
}

// =============================================================================
//  END OF SKETCH
// =============================================================================

/*
 * ============================================================
 *  TUNING GUIDE  –  READ BEFORE FIRST FLIGHT
 * ============================================================
 *
 *  PHASE 1 – Bench test (NO PROPS)
 *  --------------------------------
 *  1. Flash sketch. Open Serial Monitor at 115200 baud.
 *  2. Watch arming output and gyro offsets. Values should be
 *     small (<50 LSB). If large, check IMU mounting / vibration.
 *  3. Gently tilt the frame; confirm angle values track correctly
 *     in Serial with the debug line un-commented.
 *  4. Confirm motor directions match Quad-X diagram above.
 *     Swap any two wires on a motor to reverse it if needed.
 *
 *  PHASE 2 – Prop clearance hover (PROPS ON, tethered or low)
 *  -----------------------------------------------------------
 *  1. Start with KP=1.0, KI=0.0, KD=0.0 (pure proportional).
 *  2. Raise BASE_THROTTLE until drone lifts ~5cm, then hold.
 *  3. Increase KP until drone oscillates, then reduce by 30%.
 *  4. Add KD to damp oscillations (5–10 is typical for 10" props).
 *  5. Add tiny KI (0.01–0.05) to eliminate steady-state drift.
 *
 *  PHASE 3 – RC Integration
 *  -------------------------
 *  Replace `setpointRoll`, `setpointPitch`, `BASE_THROTTLE`
 *  with values from your RC receiver (PPM/SBUS/PWM).
 *  Add a Yaw rate PID using gyroZ.
 *
 *  SAFETY REMINDERS
 *  -----------------
 *  • Always test with props OFF first.
 *  • Keep a kill-switch (prop-blocker / ESC disarm) accessible.
 *  • The 45° tilt kill-switch triggers a hard lock; power-cycle
 *    to re-arm. This is intentional.
 *  • 4S + 1045 props = 3–4 kg total thrust. RESPECT the hardware.
 * ============================================================
 */
