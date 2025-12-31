// 情感交互人形机器人 - Arduino 下位机代码（硬件控制完整版）
// 适配：MG996R舵机 + 三色LED灯
// 指令解析：接收Python发送的 "舵机指令,LED指令" ，逗号分隔
// 接线：舵机SIG=9 | 红灯=10 | 绿灯=11 | 黄灯=12 | 所有GND共接

#include <Servo.h>

// 初始化舵机对象
Servo robotServo;
const int servoPin = 9;          // 舵机信号引脚
const int redLedPin = 10;        // 红灯引脚（消极情绪）
const int greenLedPin = 11;      // 绿灯引脚（积极情绪）
const int yellowLedPin = 12;     // 黄灯引脚（中性情绪）
int servoAngle = 90;             // 舵机初始角度：中立姿态

void setup() {
  // 1. 舵机初始化
  robotServo.attach(servoPin);
  robotServo.write(90);          // 启动时恢复中立姿态
  delay(1000);
  
  // 2. LED灯引脚初始化（输出模式）
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  turnOffAllLeds();              // 启动时关闭所有灯
  digitalWrite(yellowLedPin, HIGH);  // 启动成功，黄灯亮
  
  // 3. 串口初始化（波特率9600，与Python上位机一致）
  Serial.begin(9600);
  Serial.println("✅ 机器人硬件初始化完成，等待上位机指令...");
}

void loop() {
  // 监听串口，接收Python发送的指令
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');  // 读取指令
    int commaIndex = cmd.indexOf(',');          // 分割舵机指令和LED指令
    
    if (commaIndex != -1) {
      char servoCmd = cmd.substring(0, commaIndex).charAt(0);  // 舵机指令
      char ledCmd = cmd.substring(commaIndex + 1).charAt(0);   // LED指令
      
      // 控制舵机动作
      controlServo(servoCmd);
      // 控制LED灯亮灭
      controlLed(ledCmd);
      
      // 反馈指令执行结果
      Serial.print("✅ 指令执行完成：舵机=");
      Serial.print(servoCmd);
      Serial.print(" | LED=");
      Serial.println(ledCmd);
    } else {
      // 指令格式错误，恢复中立姿态+黄灯
      Serial.println("❌ 指令格式错误，恢复中立姿态");
      robotServo.write(90);
      turnOffAllLeds();
      digitalWrite(yellowLedPin, HIGH);
    }
  }
}

// 控制舵机转动
void controlServo(char cmd) {
  switch(cmd) {
    case 'H': servoAngle = 45;  // 开心：摆手（45°）
      break;
    case 'A': servoAngle = 135; // 生气：低头（135°）
      break;
    case 'S': servoAngle = 0;   // 难过：垂臂（0°）
      break;
    case 'N': servoAngle = 90;  // 中立：初始姿态（90°）
      break;
    default: servoAngle = 90;   // 未知指令，恢复中立
      break;
  }
  robotServo.write(servoAngle);
  delay(1500);  // 等待舵机转动完成
}

// 控制LED灯亮灭
void controlLed(char cmd) {
  turnOffAllLeds();  // 先关闭所有灯，避免多灯同时亮
  switch(cmd) {
    case 'G': digitalWrite(greenLedPin, HIGH);  // 绿灯：积极情绪
      break;
    case 'R': digitalWrite(redLedPin, HIGH);    // 红灯：消极情绪
      break;
    case 'Y': digitalWrite(yellowLedPin, HIGH); // 黄灯：中性情绪
      break;
    default: digitalWrite(yellowLedPin, HIGH);  // 未知指令，黄灯
      break;
  }
}

// 关闭所有LED灯
void turnOffAllLeds() {
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
}
