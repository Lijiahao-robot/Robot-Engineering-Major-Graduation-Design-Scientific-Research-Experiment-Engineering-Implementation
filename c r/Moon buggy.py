class LunarRover:
    def __init__(self, name):
        self.name = name
        self.speed = 0  
        self.direction = 0  
        self.status = "standby"  
  def start(self):
        self.status = "standby"
        print(f"✅ {self.name} ")

    # 直线移动
    def move_forward(self, speed=0.5):
        self.speed = speed
        self.status = "moving"
        print(f"🚀 {self.name} ：{self.speed} m/s")

    # 转向控制
    def turn(self, direction):
        self.direction = direction
        self.status = "turning"
        print(f"🔄 {self.name} 转向，角度：{self.direction}°")

    # 停止移动
    def stop(self):
        self.speed = 0
        self.status = "stopped"
        print(f"🛑 {self.name} 月球车停止移动")

# 测试代码（直接运行即可）
if __name__ == "__main__":
    yutu3 = LunarRover("玉兔三号")  # 实例化月球车
    yutu3.start()
    yutu3.move_forward(speed=0.3)
    yutu3.turn(direction=30)
    yutu3.move_forward(speed=0.2)
    yutu3.stop()
