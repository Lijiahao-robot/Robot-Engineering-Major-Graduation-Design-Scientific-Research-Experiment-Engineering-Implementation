from navigation.nav import Navigator
from vision.detect import Vision
from voice.voice_cmd import VoiceControl
from arm.ik import Arm
from control.motor import Motor

nav = Navigator()
vision = Vision()
voice = VoiceControl()
arm = Arm()
motor = Motor()

while True:
    cmd = voice.listen()

    if "forward" in cmd:
        motor.forward()

    elif "stop" in cmd:
        motor.stop()

    elif "find object" in cmd:
        x, y = vision.find_object()
        if x is not None:
            t1, t2 = arm.move_to(x, y)
            print("抓取角度:", t1, t2)

    elif "go to" in cmd:
        target = (2, 3)
        path = nav.plan(target)
        nav.follow(path)
