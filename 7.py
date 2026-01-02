class Motor:
    def forward(self): pass
    def stop(self): pass

class Sensor:
    def distance(self): return 100

class Robot:
    def __init__(self):
        self.motor = Motor()
        self.sensor = Sensor()

    def loop(self):
        if self.sensor.distance() < 20:
            self.motor.stop()
        else:
            self.motor.forward()

bot = Robot()
bot.loop()
