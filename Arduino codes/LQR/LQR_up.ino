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
const float CNT_PER_RAD_NIDEC = 400 / (2.0 * PI);  // ≈ 63.66
const float CNT_PER_RAD_ROD = 1440 / (2.0 * PI);      // ≈ 229.18

// Conversion rad to deg
const float DEG2RAD = PI / 180.0;

// Safety limits (in degrees)
const float base_limit = 180.0 * DEG2RAD;
const float thetaLQR = 20.0 * DEG2RAD;

// Voltage-to-PWM conversion factor
const float V_2_PWM = 21.25;

// Control gains - LQR
float K1 = -0.51;
float K2 = 12;
float K3 = -0.31;
float K4 = 0.88;

// Variables to store encoder readings, PWM output, and accumulated states
float rod_position = -PI;    // Current angular position of the pendulum rod (incremental encoder counts)
float rod_speed = 0;
float nidec_position = 0;  // Current angular position of the NIDEC motor
float nidec_speed = 0;
int NIDEC_count = 0;  // Raw encoder count for NIDEC motor in the current sample
int ROD_count = 0;    // Raw encoder count for pendulum rod in the current sample
float u = 0;

// Sampling time variables
float Ts = 0.01, currentT = 0.0, previousT = 0.0;

void setup() {
  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, HIGH);

  Serial.begin(115200);  // Start Serial Monitor for debugging

  // Initialize motor and encoders
  NIDECsetup();
  ENCsetup();

  // Wait until the rod encoder has moved enough counts before starting
  while (abs(rod_position) > thetaLQR) {
    rod_position = remainder(-PI + ROD_ENC.getCount()/CNT_PER_RAD_ROD, 2.0*PI);
  }  
  nidec_position = -NIDEC_ENC.getCount()/CNT_PER_RAD_NIDEC;

  NIDEC_ENC.clearCount();
  ROD_ENC.clearCount();

  previousT = millis();
  digitalWrite(INTERNAL_LED, LOW);
}

void loop() {

  currentT = millis();  // Current time in ms
  if ((currentT - previousT) / 1000.0 >= Ts) { // Run control loop every Ts seconds
    previousT = currentT;

    readEncoders();

    // Wire safety
    if (abs(nidec_position) < base_limit && abs(rod_position) < thetaLQR) {
      digitalWrite(BRAKE, HIGH);
      u = -(K1*nidec_position + K2*rod_position + K3*nidec_speed + K4*rod_speed);
      MOTORcmd(u * V_2_PWM);
    } else {
      MOTORcmd(0);
      digitalWrite(BRAKE, LOW);
    }

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

void readEncoders() {
  NIDEC_count = NIDEC_ENC.getCount();
  ROD_count = ROD_ENC.getCount();

  nidec_speed = -NIDEC_count / (Ts * CNT_PER_RAD_NIDEC);
  rod_speed = ROD_count / (Ts * CNT_PER_RAD_ROD);

  nidec_position += nidec_speed * Ts;
  rod_position += rod_speed * Ts;
  rod_position = remainder(rod_position, 2.0*PI);

  NIDEC_ENC.clearCount();
  ROD_ENC.clearCount();
}
