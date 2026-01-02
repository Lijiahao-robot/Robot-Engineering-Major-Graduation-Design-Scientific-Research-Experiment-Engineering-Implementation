import speech_recognition as sr

def action(cmd):
    if "forward" in cmd:
        print("Go forward")
    elif "left" in cmd:
        print("Turn left")
    elif "stop" in cmd:
        print("Stop")

r = sr.Recognizer()
mic = sr.Microphone()

while True:
    with mic as source:
        audio = r.listen(source)
    try:
        text = r.recognize_google(audio)
        action(text.lower())
    except:
        pass
