import speech_recognition as sr

def move(cmd):
    if "forward" in cmd:
        print("Robot moving forward")
    elif "left" in cmd:
        print("Robot turning left")
    elif "stop" in cmd:
        print("Robot stopped")

r = sr.Recognizer()
mic = sr.Microphone()

while True:
    with mic as source:
        print("Listening...")
        audio = r.listen(source)

    try:
        text = r.recognize_google(audio, language="en-US")
        print("You said:", text)
        move(text.lower())
    except:
        print("Could not understand")
