#include <BasicLinearAlgebra.h>
using namespace BLA;

// Include the encoder library for ESP32, which uses built-in hardware counters
#include <ESP32Encoder.h>

// Pin for the onboard ESP32 LED (blue LED on GPIO 2)
#define INTERNAL_LED 2

// PWM configuration for NIDEC motor driver
#define NIDEC_TIMER_BIT 8      // PWM resolution: 8-bit (0–255)
#define NIDEC_BASE_FREQ 20000  // PWM frequency: 20 kHz (to avoid audible noise)

// NIDEC motor control pins
#define BRAKE 18        // Brake pin (Start/Stop control)
#define NIDEC_PWM 19    // PWM output pin
#define DIR 23          // Direction pin (Forward/Reverse)
#define NCHA 13         // Motor encoder channel A
#define NCHB 5          // Motor encoder channel B
#define NIDEC_PWM_CH 1  // PWM channel used for ESP32 LEDC

// Encoder pins for the pendulum rod
#define RCHA 32  // Rod encoder channel A
#define RCHB 33  // Rod encoder channel B


// Encoder objects
ESP32Encoder NIDEC_ENC;  // Encoder for the NIDEC motor
ESP32Encoder ROD_ENC;    // Encoder for the pendulum rod

// Conversion rad to deg
const float RAD2DEG = 180.0f / PI;  // ≈ 57.2958

// Safety limits (in degrees)
const float BASE_LIMIT_DEG = 160.0f;  // Max angle for the NIDEC motor (±)
const float ROD_LIMIT_DEG = 20.0f;    // Max angle for the pendulum (±)

// Encoder counts per revolution (adjust according to your measurement!)
const float CPR_NIDEC = 400.0f;  // Motor encoder (100 PPR ×4 = 400)
const float CPR_ROD = 1440.0f;   // Rod encoder (360 PPR ×4 = 1440)

// Counts per radian (used to convert counts → radians)
const float CNT_PER_RAD_NIDEC = CPR_NIDEC / (2.0f * PI);  // ≈ 63.66
const float CNT_PER_RAD_ROD = CPR_ROD / (2.0f * PI);      // ≈ 229.18

// Voltage-to-PWM conversion factor
const float V_2_PWM = 21.25;

// LQR control gains
const float K1 = 0.582;
const float K2 = -3.67;
const float K3 = 0.156;
const float K4 = -0.129;

// Variables to store encoder readings, PWM output, and accumulated states
float rod_position = 0;    // Current angular position of the pendulum rod (incremental encoder counts)
float nidec_position = 0;  // Current angular position of the NIDEC motor
float nidec_integral = 0;
float nidec_speed = 0;
float rod_speed = 0;
int NIDEC_count = 0;  // Raw encoder count for NIDEC motor in the current sample
int ROD_count = 0;    // Raw encoder count for pendulum rod in the current sample
float u = 0;
float disturbance = 0;

// Sampling time variables
float Ts = 0.01, currentT = 0.0, previousT = 0.0;

// Rand aplitude
float rand_Amp = 0.7;

// ===== Model dimensions =====
constexpr int n = 4;  // Number of states
constexpr int m = 2;  // Number of measurements
constexpr int l = 1;  // Number of inputs

// ===== State vectors =====
// x = [nidec_position, rod_position, nidec_speed, rod_speed]^T
BLA::Matrix<4, 1> x = {
  0,
  0,
  0,
  0
};

BLA::Matrix<4, 1> x_pred;

// Model input vector
BLA::Matrix<1, 1> U = {
  0
};

// ===== Estimation error covariance =====
BLA::Matrix<4, 4> P = {
  10,   0,    0,    0,
  0,    10,   0,    0,
  0,    0,    10,   0,
  0,    0,    0,    10
};

BLA::Matrix<4, 4> P_pred;

// ===== Discrete-time model =====
// Zero-order-hold discretization with Ts = 0.01 s
BLA::Matrix<4, 4> A = {
  1.000,  0.00423,  0.00978, 0.0000273,
  0.000,  0.992,    0.000127, 0.00995,
  0.000,  0.846,    0.955,    0.00546,
  0.000, -1.68,     0.0253,   0.989
};

BLA::Matrix<4, 1> B = {
   0.00337,
  -0.00191,
   0.674,
  -0.381
};

// ===== Measurement matrix =====
// Only the NIDEC and rod positions are directly measured.
BLA::Matrix<2, 4> H = {
  1, 0, 0, 0,
  0, 1, 0, 0
};

// ===== Process noise covariance =====
BLA::Matrix<4, 4> Q = {
  1,    0,     0,      0,
  0,    1,     0,      0,
  0,    0,     10000,   0,
  0,    0,     0,      50000
};

// ===== Measurement noise covariance =====
BLA::Matrix<2, 2> R = {
  10,    0,
  0,     50
};

// ===== Measurement vector =====
// y = [nidec_position, rod_position]^T
BLA::Matrix<2, 1> y = {
  0,
  0
};

// ===== Kalman gain =====
BLA::Matrix<4, 2> K;


// MAIN SETUP: Runs once on startup/reset
void setup() {
  Serial.begin(115200);  // Start Serial Monitor for debugging

  // Initialize motor and encoders
  NIDECsetup();
  ENCsetup();
}

// MAIN LOOP: Runs repeatedly after setup
void loop() {

  currentT = millis()/1000.0;  // Current time
  if (currentT - previousT >= Ts) { // Run control loop every Ts seconds
    previousT = currentT;

    KalmanFilter();

    // Compute control law (state feedback)
    u = -(K1*nidec_position + K2*rod_position + K3*nidec_speed + K4*rod_speed);
    
    // if (currentT >= 10.0 && currentT <= 11.0) disturbance = 1;
    // else disturbance = 0;
    u += disturbance;

    MOTORcmd(u * V_2_PWM);

  }

  Print();

}

// NIDEC MOTOR SETUP
void NIDECsetup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);  // Enable brake initially

  pinMode(DIR, OUTPUT);
  ledcSetup(NIDEC_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC_PWM, NIDEC_PWM_CH);
  MOTORcmd(0);  // Start with motor stopped

  // Setup motor encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  NIDEC_ENC.attachFullQuad(NCHA, NCHB);
  NIDEC_ENC.clearCount();
}

// ROD ENCODER SETUP
void ENCsetup() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  ROD_ENC.attachFullQuad(RCHA, RCHB);
  ROD_ENC.clearCount();
}

// MOTOR COMMAND FUNCTION: The code makes an adjustment to the PWM because the driver hardware interprets the inverted values
void MOTORcmd(int sp) {
  // Set direction
  if (sp < 0) {
    digitalWrite(DIR, LOW);
    sp = -sp;  // Make speed positive
  } else {
    digitalWrite(DIR, HIGH);
  }

  // Apply PWM signal (inverted due to motor driver configuration)
  ledcWrite(NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void KalmanFilter() {

  // Fill the measurement vector with raw sensor measurements
  y(0) = -NIDEC_ENC.getCount()/CNT_PER_RAD_NIDEC;
  y(1) = remainder(ROD_ENC.getCount()/CNT_PER_RAD_ROD, 2.0*PI);

  // Use the voltage applied during the previous sampling interval
  U(0) = u;

  // State prediction
  x_pred = A * x + B * U;

  // Error covariance prediction
  P_pred = A * P * ~A + Q;

  // Innovation covariance
  BLA::Matrix<2, 2> S = H * P_pred * ~H + R;

  // Inverse of the innovation covariance
  BLA::Matrix<2, 2> S_inv;
  Invert(S, S_inv);

  // Kalman gain
  K = P_pred * ~H * S_inv;

  // Measurement innovation
  BLA::Matrix<2, 1> innovation;
  innovation = y - H * x_pred;

  // State update
  x = x_pred + K * innovation;

  // Error covariance update
  P = P_pred - K * H * P_pred;

  // ===== Updated variables =====
  nidec_position = x(0);
  rod_position   = x(1);
  nidec_speed = x(2);
  rod_speed   = x(3);

}

void Print() {
  Serial.print(currentT);
  Serial.print(" ");
  Serial.print(nidec_position, 6);
  Serial.print(" ");
  Serial.print(rod_position, 6);
  Serial.print(" ");
  Serial.print(nidec_speed, 6);
  Serial.print(" ");
  Serial.print(rod_speed, 6);
  Serial.print(" ");
  Serial.print(u-disturbance, 6);
  Serial.print(" ");
  Serial.println(disturbance);
}