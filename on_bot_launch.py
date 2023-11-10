from subprocess import Popen
import logging
from time import sleep

logging.basicConfig(level=logging.NOTSET)
handle = "sdp6_calibration.on_bot"

logger = logging.getLogger(handle)

logger.info("Starting calibration process on turtlebot...")

logger.info("Launching roscore...")
Popen(
    ["roscore"]
)

logger.info("Launching turtlebot3_bringup as soon as roscore is running...")
Popen(# roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ["roslaunch", "turtlebot3_bringup", "turtlebot3_robot.launch", "--wait"]
)
