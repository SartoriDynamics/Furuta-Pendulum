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

// Counts per radian (used to convert counts → radians)
const double CNT_PER_RAD_NIDEC = 400 / (2.0 * PI);  // ≈ 63.66
const double CNT_PER_RAD_ROD = 1440 / (2.0 * PI);      // ≈ 229.18

// Conversion rad to deg
const double DEG2RAD = PI / 180.0;

// Safety limits (in degrees)
const double base_limit = 180.0 * DEG2RAD;
const double thetaLQR = 20.0 * DEG2RAD;

// Voltage-to-PWM conversion factor
const double V_2_PWM = 21.25;

// LQR control gains
const double K1 = -0.537;
const double K2 = 16.8;
const double K3 = -0.326;
const double K4 = 1.34;


// Variables to store encoder readings, PWM output, and accumulated states
double rod_position = -PI;    // Current angular position of the pendulum rod (incremental encoder counts)
double rod_speed = 0;
double nidec_position = 0;  // Current angular position of the NIDEC motor
double nidec_speed = 0;
int NIDEC_count = 0;  // Raw encoder count for NIDEC motor in the current sample
int ROD_count = 0;    // Raw encoder count for pendulum rod in the current sample
double u = 0, disturbance = 0;

// Sampling time variables
double Ts = 0.01, currentT = 0.0, previousT = 0.0;

// Rand aplitude
double rand_Amp = 0.0;

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
  1.000, 0.00427,  0.00978, 0.0000151,
  0.000, 1.01,   -0.000128, 0.0100,
  0.000, 0.853,   0.955,    0.00302,
  0.000, 1.70,   -0.0255,   1.01
};

BLA::Matrix<4, 1> B = {
  0.00338,
  0.00192,
  0.675,
  0.384
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
  0,    0,     40000,   0,
  0,    0,     0,      20000
};

// ===== Measurement noise covariance =====
BLA::Matrix<2, 2> R = {
  1,    0,
  0,     1
};

// ===== Measurement vector =====
// y = [nidec_position, rod_position]^T
BLA::Matrix<2, 1> y = {
  0,
  0
};

// ===== Kalman gain =====
BLA::Matrix<4, 2> K;


void setup() {
  Serial.begin(115200);  // Start Serial Monitor for debugging

  // Initialize motor and encoders
  NIDECsetup();
  ENCsetup();

  // Wait until the rod encoder has moved enough counts before starting
  while (abs(rod_position) > thetaLQR) {
    rod_position = remainder(-PI + ROD_ENC.getCount()/CNT_PER_RAD_ROD, 2.0*PI);
  }  
  nidec_position = -NIDEC_ENC.getCount()/CNT_PER_RAD_NIDEC;

  // Initialize the Kalman filter with the measured initial positions
  x(0) = nidec_position;
  x(1) = rod_position;
  x(2) = 0.0;
  x(3) = 0.0;
}

void loop() {

  currentT = micros()/1000000.0;  // Current time
  if (currentT - previousT >= Ts) { // Run control loop every Ts seconds
    previousT = currentT;

    KalmanFilter();

    // Wire safety
    if (abs(y(0)) < base_limit && abs(y(1)) < thetaLQR) {

      digitalWrite(BRAKE, HIGH);
      u = -(K1*nidec_position + K2*rod_position + K3*nidec_speed + K4*rod_speed);
      // if (nidec_speed==0) u += random(rand_Amp)-(rand_Amp/2); // sometimes the motor stops

      if (currentT >= 20.0 && currentT <= 21.0) disturbance = 1;
      // else disturbance = 0;
      u += disturbance;

      MOTORcmd(u * V_2_PWM);

    } else {
      MOTORcmd(0);
      digitalWrite(BRAKE, LOW);
    }

    Print();

  }

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
  y(1) = remainder(-PI + ROD_ENC.getCount()/CNT_PER_RAD_ROD, 2.0*PI);

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
