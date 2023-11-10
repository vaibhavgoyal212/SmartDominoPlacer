from motorControllerHelper.iotools import MotorControl

class Motors(object):
    def __init__(self):
        self.mc = MotorControl()

    def start_motor(self, id: int, speed: int): 
        self.mc.setMotor(id, speed)

    def stop_motor(self, id: int):
        self.mc.stopMotor(id)

    def stop_motors(self):
        self.mc.stopMotors()
