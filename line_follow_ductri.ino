// Khai báo chân cảm biến hồng ngoại (IR)
#define leftIR   A0   
#define centerIR A1   
#define rightIR  A2   

// Khai báo chân điều khiển động cơ
#define ENA  6        // PWM bên phải
#define IN1  8       
#define IN2  7
#define IN3  10       
#define IN4  9
#define ENB  11       // PWM bên trái

// Thông số PID
float Kp = 3, Kd = 45, Ki = 0;       

// Tốc độ cơ sở của mỗi bánh xe
int baseSpeedRight = 175;
int baseSpeedLeft = 165;
int maxSpeed = 255;

// Biến PID
float error = 0, lastError = 0, integral = 0;
unsigned long lastTime = 0;


void setup() {
  Serial.begin(9600);
  pinMode(leftIR, INPUT);
  pinMode(centerIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  lastTime = millis();
}

void loop() {
  errCal();       
  PIDControl();  
  delay(20);      
}
void errCal() {
  int LS[3] = {0, 0, 0};  
  int weights[3] = {-35, 0, 35}; // Trọng số sai số
  if (digitalRead(leftIR) == HIGH)   LS[0] = 1;
  if (digitalRead(centerIR) == HIGH) LS[1] = 1;
  if (digitalRead(rightIR) == HIGH)  LS[2] = 1;

  int total = 0;  
  int count = 0;        
  for (int i = 0; i < 3; i++) {
    total += LS[i] * weights[i];
    count += LS[i];
  }
  if (count == 0 ) {
    error = lastError; // giữ nguyên lối cũ
    return;
  }
  error = (float)total / count;  // Sai số trung bình có trọng số
}

// Hàm điều khiển tốc độ động cơ dựa vào PID
void PIDControl() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; 
  lastTime = currentTime;

  integral += error;
  float derivative = (error - lastError) / dt;  // Đạo hàm sai số
  lastError = error;

  // Tính đầu ra của bộ điều khiển PID
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Điều chỉnh tốc độ từng bánh xe
  int leftSpeed  = constrain(baseSpeedLeft + output,  -maxSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeedRight - output, -maxSpeed, maxSpeed);

  // Cập nhật tốc độ cho động cơ
  setMotor(ENA, IN1, IN2, rightSpeed); 
  setMotor(ENB, IN3, IN4, leftSpeed);  
}
void setMotor(int pwmPin, int in1, int in2, int speed) {
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  }
}
