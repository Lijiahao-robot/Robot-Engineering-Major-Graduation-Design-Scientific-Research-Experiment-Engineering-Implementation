#include <SoftwareSerial.h>

// 电机引脚定义
#define MOTOR_LEFT_PWM 5
#define MOTOR_LEFT_DIR 6
#define MOTOR_RIGHT_PWM 9
#define MOTOR_RIGHT_DIR 10

// 速度参数
float linear_x = 0.0;
float angular_z = 0.0;
const float WHEEL_RADIUS = 0.03; // 轮子半径 3cm
const float WHEEL_BASE = 0.16;   // 轮距 16cm

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCmd(cmd); // 解析速度指令
    calculateMotorSpeed(); // 计算左右轮速度
  }
}

// 解析串口指令 "v:x,w:z"
void parseCmd(String cmd) {
  int v_index = cmd.indexOf("v:");
  int w_index = cmd.indexOf("w:");
  if (v_index != -1 && w_index != -1) {
    linear_x = cmd.substring(v_index+2, w_index-1).toFloat();
    angular_z = cmd.substring(w_index+2).toFloat();
  }
}

// 运动学逆解：线速度、角速度转左右轮转速
void calculateMotorSpeed() {
  float left_speed = (linear_x - angular_z * WHEEL_BASE / 2) / WHEEL_RADIUS;
  float right_speed = (linear_x + angular_z * WHEEL_BASE / 2) / WHEEL_RADIUS;
  
  // 转换为 PWM 值（-255~255）
  int left_pwm = constrain(left_speed * 100, -255, 255);
  int right_pwm = constrain(right_speed * 100, -255, 255);
  
  // 控制电机方向
  digitalWrite(MOTOR_LEFT_DIR, left_pwm > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_DIR, right_pwm > 0 ? HIGH : LOW);
  
  // 控制电机转速
  analogWrite(MOTOR_LEFT_PWM, abs(left_pwm));
  analogWrite(MOTOR_RIGHT_PWM, abs(right_pwm));
}
