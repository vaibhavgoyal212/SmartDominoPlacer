import math
import sys
from time import time, sleep
from motorControllerHelper.motors import Motors 

class PiMotorController():
    def __init__(self) -> None:
        self.mc = Motors() # motor controller
        # Pushers
        self.push_motors = [1, 2, 3, 4, 5]
        self.push_speeds = [
            {True: 100, False: -50},
            {True: 100, False: -50},
            {True: 100, False: -50},
            {True: 100, False: -50},
            {True: 100, False: -50}
            ]
        self.push_durations_ms = [
            {True: 420, False: 300},
            {True: 420, False: 300},
            {True: 420, False: 300},
            {True: 420, False: 300},
            {True: 420, False: 300},
            ]
        # Lifters
        self.lift_motors = [0]
        self.lift_speeds = [{True: 70, False: -70}]
        self.lift_durations_ms = [{True: 1700, False: 1380}]
    
    def stop_motors(self):
        self.mc.stop_motors()
    
    def _move_pushers(self, indexes: "list[int]", pattern: "list[bool]", reversed: bool = False):
        # Reverse pattern if necessary
        if reversed:
            for i in indexes:
                pattern[i] = not pattern[i]
        # Start motors
        for i in indexes:
            motor_id = self.push_motors[i]
            speed = self.push_speeds[i][pattern[i]]
            self.mc.start_motor(motor_id, speed)
        # Keep running motors
        start_time = time()
        inProgress = True
        while inProgress:
            inProgress = False
            for i in indexes:
                motor_id = self.push_motors[i]
                duration = self.push_durations_ms[i][pattern[i]] / 1000
                if time() >= start_time + duration:
                    # Stop motor if its duration is over
                    self.mc.stop_motor(motor_id)
                else:
                    # Keep looping while at least one motor needs to be run 
                    inProgress = True
            # sleep(0.01)

    def _move_lifters(self, direction: bool):
        # Start motors
        for i in range(len(self.lift_motors)):
            motor_id = self.lift_motors[i]
            speed = self.lift_speeds[i][direction]
            self.mc.start_motor(motor_id, speed)
        # Keep running motors
        start_time = time()
        inProgress = True
        while inProgress:
            inProgress = False
            for i in range(len(self.lift_motors)):
                motor_id = self.lift_motors[i]
                duration = self.lift_durations_ms[i][direction] / 1000
                if time() >= start_time + duration:
                    # Stop motor if its duration is over
                    self.mc.stop_motor(motor_id)
                else:
                    # Keep looping while at least one motor needs to be run 
                    inProgress = True
            # sleep(0.01)

    def place_row(self, pattern: "list[bool]"):
        if len(self.push_motors) < len(pattern): raise Exception("Missing push ids")
        if len(self.push_speeds) < len(pattern): raise Exception("Missing push speeds")
        if len(self.push_durations_ms) < len(pattern): raise Exception("Missing push durations")
        
        max_at_same_time = 2 # How many motors to move at the same time
        for i in range(math.ceil(len(pattern)/max_at_same_time)):
            indexes = list(range(i*max_at_same_time, min((i+1)*max_at_same_time, len(pattern))))
            self._move_pushers(indexes, pattern) # Push
            self._move_pushers(indexes, pattern, reversed=True) # Center after push

    def raise_wall(self):
        self._move_lifters(True)
    
    def lower_wall(self):
        self._move_lifters(False)
    

def debug_run(self):
    controller = PiMotorController()
    controller.place_row(pattern=[True, True, True, True, True])
    controller.place_row(pattern=[False, False, False, False, False])


if __name__ == '__main__':
    args = sys.argv
    func_name = args.pop(0)
    controller = PiMotorController()
    if func_name == "place_row":
        binary_pattern = args.pop(0)
        pattern = [bool(x) for x in binary_pattern]
        controller.place_row(pattern)
    else:
        getattr(controller, func_name)()
    
    #controller.raise_wall()
    #controller.lower_wall()
