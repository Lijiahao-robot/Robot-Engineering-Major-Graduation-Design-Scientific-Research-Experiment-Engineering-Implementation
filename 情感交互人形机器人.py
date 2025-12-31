# 情感交互人形机器人 - 硬件适配完整版（Python上位机）
# 适配硬件：Arduino UNO + MG996R舵机 + 三色LED灯
# 闭环流程：语音采集→情感识别→语音回应→舵机动作→LED联动→日志保存→情绪记忆
# 依赖库安装：pip install speechrecognition pyttsx3 requests pyserial python-Levenshtein

import speech_recognition as sr
import pyttsx3
import requests
import serial
import time
import sys
from datetime import datetime

# ---------------------- 【必改配置】适配你的硬件/API ----------------------
# 百度情感分析API（免费申请：https://ai.baidu.com/ai-doc/NLP/zk6z52g9z，1分钟申请完成）
API_KEY = "你的API_KEY"
SECRET_KEY = "你的SECRET_KEY"
BAUD_RATE = 9600
SERIAL_PORT = "COM3"  # Windows：COM3/COM4 | Mac/Linux：/dev/tty.usbmodem14101
LOG_PATH = "emotion_robot_log.txt"  # 交互日志保存路径

# ---------------------- 情感配置（情绪→回应+舵机指令+LED指令）----------------------
# 指令规则：舵机指令+H=开心(45°) | A=生气(135°) | S=难过(0°) | N=中立(90°)
# LED指令：G=绿灯（积极）| R=红灯（消极）| Y=黄灯（中性）
emotion_config = {
    "positive": [
        "哇，听你这么开心，我也跟着超快乐！",
        "太治愈啦～ 这份幸福感我必须接住！",
        "哈哈哈，你今天心情也太好了吧，太为你开心啦！",
        "H", "G"
    ],
    "negative": [
        "抱抱你，委屈都可以跟我说，我永远陪着你～",
        "别气别气，深呼吸，不值得为小事影响心情呀",
        "我懂这种难过的感觉，慢慢说，我一直听着你",
        "A", "R"
    ],
    "neutral": [
        "好的，我明白啦，还有什么想跟我分享的吗？",
        "嗯嗯，我听懂了，你继续说～",
        "收到，有没有其他想跟我聊的呀？",
        "N", "Y"
    ]
}

# ---------------------- 全局初始化（硬件+软件双配置）----------------------
# 1. 语音组件初始化（拟人化语速/音量）
recognizer = sr.Recognizer()
recognizer.energy_threshold = 4000  # 抗杂音优化
engine = pyttsx3.init()
engine.setProperty('rate', 145)  # 接近人类说话速度
engine.setProperty('volume', 1.0)

# 2. 串口初始化（连接Arduino，控制舵机+LED）
try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    time.sleep(2)  # 等待Arduino启动，避免指令丢失
    print("✅ Arduino串口连接成功！舵机+LED灯已就绪")
except Exception as e:
    print(f"❌ 串口连接失败：{e}")
    print("💡 提示：检查串口是否占用、硬件是否通电、接线是否正确")
    sys.exit(1)

# ✨ 情绪记忆相关 ✨ 新增：记录最近3次情绪，滚动记忆
recent_emotions = []
MAX_MEMORY_COUNT = 3  # 最大记忆条数

# ---------------------- 核心工具函数（完整无遗漏，异常全捕获）----------------------
def speak(text: str) -> None:
    """语音合成：机器人说话，捕获合成异常"""
    try:
        engine.say(text)
        engine.runAndWait()
    except Exception as e:
        print(f"【语音合成错误】：{e}")

def save_interaction_log(user_text: str, emotion: str, robot_response: str) -> None:
    """保存交互日志，用于项目复盘（专升本项目加分项）"""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_content = f"[{now}] 用户输入：{user_text} | 识别情绪：{emotion} | 最近情绪：{recent_emotions} | 机器人回应：{robot_response}\n"
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(log_content)
    print(f"📝 交互日志已保存：{log_content.strip()}")

# ✨ 情绪记忆相关 ✨ 新增：根据最近情绪，生成贴合的记忆回应
def get_memory_response(emotion_type: str) -> str:
    """情绪记忆回应：连续情绪追加话术，让回应更有温度"""
    if len(recent_emotions) < 2:
        return ""  # 不足2条记忆，不追加话术
    
    # 连续3次积极情绪：加倍鼓励
    if all(emo == "positive" for emo in recent_emotions):
        return " 我发现你最近心情一直都超好，这份快乐一定要一直保持呀～"
    # 连续3次消极情绪：深度安慰
    elif all(emo == "negative" for emo in recent_emotions):
        return " 我知道你最近一直有点难，没关系，不管怎么样，我都会一直陪着你倾听你～"
    # 连续2次相同情绪：适度关心
    elif recent_emotions[-1] == recent_emotions[-2] == emotion_type:
        if emotion_type == "positive":
            return " 你今天真的太开心啦，看着你快乐，我也超幸福～"
        elif emotion_type == "negative":
            return " 是不是还是有点不舒服？没关系，慢慢说，我听你倾诉～"
    # 情绪波动：温柔安抚
    else:
        return " 我看你心情有点起伏，不管开心还是难过，都可以跟我说说呀～"

def get_baidu_emotion(text: str) -> str:
    """调用百度情感API，识别情绪（positive/negative/neutral），准确率远超轻量库"""
    try:
        # 第一步：获取API访问令牌
        token_url = f"https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id={API_KEY}&client_secret={SECRET_KEY}"
        token_res = requests.get(token_url).json()
        access_token = token_res["access_token"]
        
        # 第二步：调用情感分析接口
        headers = {"Content-Type": "application/json"}
        params = {"access_token": access_token}
        data = {"text": text}
        res = requests.post("https://aip.baidubce.com/rpc/2.0/nlp/v1/emotion", headers=headers, params=params, json=data).json()
        return res["items"][0]["sentiment"]  # 返回情绪类型
    except Exception as e:
        print(f"【百度情感API错误】：{e}")
        return "neutral"  # 出错兜底为中性情绪

def control_hardware(servo_cmd: str, led_cmd: str) -> None:
    """发送指令给Arduino，控制舵机动作+LED灯亮灭"""
    try:
        if ser.isOpen():
            # 指令格式：舵机指令,LED指令（逗号分隔，方便Arduino解析）
            control_cmd = f"{servo_cmd},{led_cmd}".encode("utf-8")
            ser.write(control_cmd)
            print(f"🤖  硬件控制指令发送成功：舵机={servo_cmd} | LED={led_cmd}")
            time.sleep(1.8)  # 等待舵机转动完成（MG996R耗时1.5秒）
        else:
            print("❌ 串口已关闭，正在重新连接...")
            ser.open()
            time.sleep(1)
            control_hardware(servo_cmd, led_cmd)
    except Exception as e:
        print(f"【硬件控制错误】：{e}")

def emotion_interaction_loop() -> None:
    """主交互闭环：监听→转文字→识情绪→记情绪→回应→控硬件→存日志"""
    global recent_emotions
    with sr.Microphone() as source:
        print("\n" + "-"*60)
        print("🔍 正在聆听...（请说话，说完停顿1秒）")
        print("💡 提示：输入 'q' 并回车，可直接安全退出程序")
        print("-"*60)
        recognizer.adjust_for_ambient_noise(source, duration=0.5)  # 适配环境噪音
        audio = recognizer.listen(source, timeout=6, phrase_time_limit=12)
    
    try:
        # 1. 语音转文字（谷歌中文接口，免费稳定）
        user_text = recognizer.recognize_google(audio, language="zh-CN")
        print(f"🗣️  你说：{user_text}")
        
        # 2. 识别情绪
        emotion_type = get_baidu_emotion(user_text)
        print(f"😀  识别到情绪：{emotion_type}")
        
        # ✨ 情绪记忆相关 ✨ 新增：更新情绪记忆，滚动保留最近3条
        recent_emotions.append(emotion_type)
        if len(recent_emotions) > MAX_MEMORY_COUNT:
            recent_emotions.pop(0)
        print(f"📝 最近情绪记忆：{recent_emotions}")
        
        # 3. 生成回应（基础回应 + 记忆追加回应，更有温度）
        responses = emotion_config[emotion_type][:3]
        base_response = responses[time.time() % 3 // 1]  # 随机选取基础回应
        memory_response = get_memory_response(emotion_type)  # 情绪记忆回应
        robot_response = base_response + memory_response
        servo_cmd = emotion_config[emotion_type][3]
        led_cmd = emotion_config[emotion_type][4]
        
        # 4. 机器人语音回应
        print(f"🤖  机器人回应：{robot_response}")
        speak(robot_response)
        
        # 5. 控制硬件（舵机+LED灯）
        control_hardware(servo_cmd, led_cmd)
        
        # 6. 保存交互日志
        save_interaction_log(user_text, emotion_type, robot_response)
        
    except sr.UnknownValueError:
        print("❌ 抱歉，我没听清你说的话，麻烦再说一遍～")
        speak("抱歉，我没听清你说的话，麻烦再说一遍～")
        control_hardware("N", "Y")  # 未听清，恢复中立姿态+黄灯
        # ✨ 情绪记忆相关 ✨ 新增：未听清，不记录情绪
    except sr.WaitTimeoutError:
        print("⌛ 聆听超时，请重新说话～")
        speak("聆听超时，请重新说话～")
        control_hardware("N", "Y")
        # ✨ 情绪记忆相关 ✨ 新增：超时，不记录情绪
    except Exception as e:
        print(f"【交互总错误】：{e}")
        control_hardware("N", "Y")
        # ✨ 情绪记忆相关 ✨ 新增：出错，不记录情绪

# ---------------------- 主运行入口（安全退出，保护硬件）----------------------
if __name__ == "__main__":
    print("🌟 情感交互人形机器人（硬件适配+情绪记忆版）启动成功！")
    print("📌 核心功能：语音→情感→情绪记忆→语音回应→舵机动作→LED联动→日志保存")
    print("📌 适配硬件：Arduino UNO + MG996R + 三色LED + 麦克风+扬声器")
    
    try:
        while True:
            user_input = input("\n👉 按下回车开始聆听，输入 'q' 退出：")
            if user_input.lower() == 'q':
                print("👋 感谢使用，机器人正在安全关闭...")
                control_hardware("N", "Y")  # 退出前，舵机恢复中立+黄灯
                speak("感谢你的陪伴，再见啦～")
                ser.close()  # 关闭串口，保护Arduino和舵机
                print("✅ 串口已关闭，硬件安全，机器人正常退出～")
                sys.exit(0)
            emotion_interaction_loop()
    except KeyboardInterrupt:
        # 快捷键 Ctrl+C 退出，同样保护硬件
        control_hardware("N", "Y")
        ser.close()
        print("\n👋 快捷键退出，机器人正常关闭～")
        sys.exit(0)
