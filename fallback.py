class FallbackManager:
    def check(self, navigator):
        if navigator.isTaskComplete():
            return False
        if navigator.getFeedback().navigation_time.sec > 30:
            return True
