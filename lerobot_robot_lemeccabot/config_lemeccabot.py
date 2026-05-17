from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig
from numpy import byte


@RobotConfig.register_subclass("le_mecca_bot")
@dataclass
class LeMeccaBotConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory={
            "wrist": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                width=480,
                height=640,
            ),
            "top": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                width=480,
                height=640,
            ),
        }
    )
    baudrate: int = 9600
    nb_motors: int = 5
    
    # TODO add in calibration
    servoStartPos = [100,170,100,100,90] # Start position for each servo.
    
    ## Servo constants are set in config class due to strong individual variability
    ## of this cheap, tweaked and factory uncalibrated material

    # TODO add in calibration
    # Servo min and max accepted position commands both for safety and servo's limit detection.
    servoMinCmd = [0,40,0,0,0]
    servoMaxCmd = [180,180,170,180,90]

    # Servo min and max potentiometer feedback
    servoMinFeedback = [50,50,50,50,50]
    servoMaxFeedback = [960,950,950,960,830]

    #PID variables
    KP = [1,1,1,1,45]
    KD = [0,0,0,0,0]
    KI = [0,0,0,0,5]
