from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from firmware.sg90 import SG90MotorsBus
from lerobot.robots import Robot
from LeMeccaBot.lerobot_robot_lemeccabot.config_lemeccabot import LeMeccaBotConfig

class LeMeccaBot(Robot):
    config_class = LeMeccaBotConfig
    name = "le_mecca_bot"

    def __init__(self, config: LeMeccaBotConfig):
        super().__init__(config)
        self.bus = SG90MotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "PTK7465", MotorNormMode.RANGE_M100_100),
                "joint_2": Motor(2, "MG90S", MotorNormMode.RANGE_M100_100),
                "joint_3": Motor(3, "MG90S", MotorNormMode.RANGE_M100_100),
                "joint_4": Motor(4, "MG90S", MotorNormMode.RANGE_M100_100),
                "joint_5": Motor(5, "MG90S", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_0.pos": float,
            "joint_0.torque": float,
            "joint_1.pos": float,
            "joint_1.torque": float,
            "joint_2.pos": float,
            "joint_2.torque": float,
            "joint_3.pos": float,
            "joint_3.torque": float,
            "joint_4.pos": float,
            "joint_4.torque": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}
    
    def action_features(self) -> dict:
        return self._motors_ft          #Torque isn't implemented yet in firmware, use position only
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def disconnect(self) -> None:
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

#Note that depending on your hardware, this may not apply. If that’s the case, you can simply leave these methods as no-ops:
# MotorBus.calibrate() an MotorBus.is_calibrated not implemented yet.
    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

# TODO 
    # @property
    # def is_calibrated(self) -> bool:
    #     return self.bus.is_calibrated

# def calibrate(self) -> None:
#     self.bus.disable_torque()
#     for motor in self.bus.motors:
#         self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

#     input(f"Move {self} to the middle of its range of motion and press ENTER....")
#     homing_offsets = self.bus.set_half_turn_homings()

#     print(
#         "Move all joints sequentially through their entire ranges "
#         "of motion.\nRecording positions. Press ENTER to stop..."
#     )
#     range_mins, range_maxes = self.bus.record_ranges_of_motion()

#     self.calibration = {}
#     for motor, m in self.bus.motors.items():
#         self.calibration[motor] = MotorCalibration(
#             id=m.id,
#             drive_mode=0,
#             homing_offset=homing_offsets[motor],
#             range_min=range_mins[motor],
#             range_max=range_maxes[motor],
#         )

#     self.bus.write_calibration(self.calibration)
#     self._save_calibration()
#     print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.write_register("CMOD", LeMeccaBotConfig.CMOD) 
            self.bus.write_register("KP", LeMeccaBotConfig.KP) 
            self.bus.write_register("KI", LeMeccaBotConfig.KI) 
            self.bus.write_register("KD", LeMeccaBotConfig.KD) 
            self.bus.write_register("servoMinCmd", LeMeccaBotConfig.servoMinCmd) 
            self.bus.write_register("servoMaxCmd", LeMeccaBotConfig.servoMaxCmd) 
            self.bus.write_register("servoMinFeedback", LeMeccaBotConfig.servoMinFeedback) 
            self.bus.write_register("servoMaxFeedback", LeMeccaBotConfig.servoMaxFeedback)


def get_observation(self) -> dict[str, Any]:
    if not self.is_connected:
        raise ConnectionError(f"{self} is not connected.")

    response = self.bus.sync_read()

    # Read arm position
    obs_dict = self.bus.sync_read("F")
    obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}

    # Read arm torque
    torque_dict = self.bus.sync_read("I")
    torque_dict = {f"{motor}.torque": val for motor, val in torque_dict.items()}
    obs_dict.update(torque_dict)

    # Capture images from cameras
    for cam_key, cam in self.cameras.items():
        obs_dict[cam_key] = cam.async_read()

    return obs_dict

def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    goal_pos = {key.removesuffix(".pos"): val for key, val in action.items()}

    # Send goal position to the arm
    self.bus.sync_write("W", goal_pos) #TODO add cmd support for sync_read

    return action

