// motor_controller.ino
// For Line Follower Robot using L298N Motor Driver
// Protocol: "M,<left_pwm>,<right_pwm>\n" where pwm ∈ [-127..127]

const int ENA = 5;   // Left motor PWM
const int IN1 = 6;   // Left motor dir1
const int IN2 = 7;   // Left motor dir2

const int ENB = 9;   // Right motor PWM
const int IN3 = 10;  // Right motor dir1
const int IN4 = 11;  // Right motor dir2

char inBuf[32];

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void setMotor(int pwm, int in1, int in2, int enPin) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enPin, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enPin, 0);
  }
}

void loop() {
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', inBuf, sizeof(inBuf)-1);
    inBuf[len] = 0;
    if (len > 0 && inBuf[0] == 'M' && inBuf[1] == ',') {
      int l = 0, r = 0;
      char *p = inBuf + 2;
      l = atoi(p);
      char *comma = strchr(p, ',');
      if (comma) r = atoi(comma+1);
      l = constrain(l, -127, 127);
      r = constrain(r, -127, 127);
      int pwmL = map(l, -127, 127, -255, 255);
      int pwmR = map(r, -127, 127, -255, 255);
      setMotor(pwmL, IN1, IN2, ENA);
      setMotor(pwmR, IN3, IN4, ENB);
    }
  }
}
