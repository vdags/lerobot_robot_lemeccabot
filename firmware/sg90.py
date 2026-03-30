from lerobot.motors.motors_bus import  MotorsBusBase, MotorCalibration, Motor,Value
import serial
import time
"""
Parallele protocole for communication with an Arduino controlling SG90 servos. 
The Arduino should be programmed to receive commands.
See './sg90_arduino/sg90_arduino.ino' for the Arduino code.

Start delimiter: "<"
End delimiter: ">"

Commands:

Connection handshake: "<C>", gets "<C, OK>"
Disconnection handshake: "<D>", gets "<D, OK>"
Read: "<R, motor_id>", gets "<R, motor_id, value>"
        throws "<R, motor_id, KO>"
Write: "<W, motor_id, value>", gets "<W, motor_id, value>"
        throws "<W, motor_id, KO>"
Enable torque: "<E, motor_id>", gets "<E, motor_id, OK>"
        throws "<E, motor_id, KO>"
Disable torque: "<T, motor_id>", gets "<T, motor_id, OK>"
        throws "<T, motor_id, KO>"
"""

BAUDRATE=9600

class SG90MotorsBus(MotorsBusBase):
    START_MARKER = ord('<')
    END_MARKER = ord('>')


    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None):
        self.port = port
        self.motors = motors
        self.calibration = calibration if calibration else {}
        self.serial = None
    
    def connect(self, handshake: bool = True) -> None:
        """Establish connection to the motors."""
        self.serial = serial.Serial(self.port, BAUDRATE)
        print ("Serial port " + self.port + " opened  Baudrate " + str(BAUDRATE))
        self.waitForArduino()

    def disconnect(self, disable_torque: bool = True) -> None:
        """Disconnect from the motors."""
        if self.serial:
            self.serial.close()

    @property
    def is_connected(self) -> bool:
        """Check if connected to the motors."""
        if self.serial.is_open:
            self.sendToArduino("<C>")
            response = self.recvFromArduino()
            if response == "C, OK":
                return True
            else:
                print(f"Unexpected response to connection check: {response}")
                return False
        else:
            print("Serial port is not open.")
            return False

    def read(self, data_name: str, motor: str) -> Value:
        """Read a value from a single motor."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<R, {motor}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<R, {motor}, KO>":
            raise ValueError(f"Failed to read value for motor {motor}")
        return response

    def write(self, data_name: str, motor: str, value: Value) -> None:
        """Write a value to a single motor."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<W, {motor}, {value}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<W, {motor}, KO>":
            raise ValueError(f"Failed to write value for motor {motor}")
        return response
        

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, Value]:
        """Read a value from multiple motors."""
        pass

    def sync_write(self, data_name: str, values: dict[str, Value]) -> None:
        """Write values to multiple motors."""
        pass


    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        """Enable torque on selected motors."""
        ##TODO add vectorisation
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<E, {motors}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<E, {motors}, KO>":
            raise ValueError(f"Failed to enable torque for motors {motors}")
        return response

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        """Disable torque on selected motors."""
        ##TODO add vectorisation
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<D, {motors}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<D, {motors}, KO>":
            raise ValueError(f"Failed to disable torque for motors {motors}")
        return response

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """Read calibration parameters from the motors."""
        pass

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        """Write calibration parameters to the motors."""
        pass

    def sendToArduino(self, msg: str) -> None:
        """
        Send a message to the Arduino, delimited by START_MARKER and END_MARKER.
        The message must include the delimiters.
        """
        self.serial.write(msg.encode('utf-8')) 

    def recvFromArduino(self) -> str:
        """
        Receive a message from the Arduino, delimited by START_MARKER and END_MARKER.
        Returns the message as a string, without the delimiters."""    
        ck = ""
        x = "z" # any value that is not an end- or startMarker
        byteCount = -1 # to allow for the fact that the last increment will be one too many
        
        # wait for the start character
        while  ord(x) != self.START_MARKER: 
            x = self.serial.read()
        
        # save data until the end marker is found
        while ord(x) != self.END_MARKER:
            if ord(x) != self.START_MARKER:
                ck = ck + x.decode("utf-8") # change for Python3
                byteCount += 1
            x = self.serial.read()
        
        return(ck)
    
    def waitForArduino(self):
        """
        wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
        it also ensures that any bytes left over from a previous message are discarded
        """
        msg = ""
        while msg.find("Arduino is ready") == -1:
            while self.serial.in_waiting == 0:
                pass
            msg = self.recvFromArduino()
            print (msg)