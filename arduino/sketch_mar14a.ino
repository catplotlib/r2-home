const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;

const int ENC1_A = 2;
const int ENC1_B = 4;
const int ENC2_A = 3;
const int ENC2_B = 7;

const float PPR1 = 180.0;
const float PPR2 = 2281.0;

volatile long enc1_count = 0;
volatile long enc2_count = 0;

String inputBuffer = "";

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readEnc2, CHANGE);

  stopMotors();
  Serial.println("Ready");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // publish encoder counts and radians every 50ms
  static unsigned long last_pub = 0;
  if (millis() - last_pub > 50) {
    float rad1 = (float)enc1_count / PPR1 * 2.0 * PI;
    float rad2 = (float)enc2_count / PPR2 * 2.0 * PI;
    Serial.print("E ");
    Serial.print(enc1_count);
    Serial.print(" ");
    Serial.print(enc2_count);
    Serial.print(" ");
    Serial.print(rad1, 4);
    Serial.print(" ");
    Serial.println(rad2, 4);
    last_pub = millis();
  }
}

void handleCommand(String msg) {
  msg.trim();
  char cmd = msg.charAt(0);

  if (cmd == 'M') {
    int s1 = msg.indexOf(' ');
    int s2 = msg.indexOf(' ', s1 + 1);
    int left  = msg.substring(s1 + 1, s2).toInt();
    int right = msg.substring(s2 + 1).toInt();
    setA(abs(left),  left  >= 0);
    setB(abs(right), right >= 0);
  } else if (cmd == 'X') {
    enc1_count = 0;
    enc2_count = 0;
    Serial.println("OK X");
  } else {
    int spd = msg.length() > 1 ? msg.substring(2).toInt() : 0;
    spd = constrain(spd, 0, 255);
    switch (cmd) {
      case 'F': forward(spd);   break;
      case 'B': backward(spd);  break;
      case 'L': turnLeft(spd);  break;
      case 'R': turnRight(spd); break;
      case 'S': stopMotors();   break;
    }
    Serial.print("OK "); Serial.println(cmd);
  }
}

void readEnc1() {
  if (digitalRead(ENC1_B) == digitalRead(ENC1_A))
    enc1_count++;
  else
    enc1_count--;
}

void readEnc2() {
  if (digitalRead(ENC2_B) == digitalRead(ENC2_A))
    enc2_count++;
  else
    enc2_count--;
}

void setA(int spd, bool fwd) {
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
}

void setB(int spd, bool fwd) {
  digitalWrite(IN3, fwd ? HIGH : LOW);
  digitalWrite(IN4, fwd ? LOW  : HIGH);
}

void forward(int spd)   { setA(spd, true);  setB(spd, true);  }
void backward(int spd)  { setA(spd, false); setB(spd, false); }
void turnLeft(int spd)  { setA(spd, false); setB(spd, true);  }
void turnRight(int spd) { setA(spd, true);  setB(spd, false); }
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}