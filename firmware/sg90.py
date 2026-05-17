from lerobot.motors.motors_bus import  MotorsBusBase, MotorCalibration, Motor,Value
import serial
import time
import re

"""
SG90 Arduino Firmware Protocol Documentation

OVERVIEW:
---------
This module implements serial communication with an Arduino controlling up to 6 SG90 servos.
The Arduino runs sg90_arduino.ino firmware with a message-based protocol using text commands.
Baud rate: 9600 | Frame format: <command, args...>

MESSAGE FORMAT:
---------------
Start delimiter: '<'
End delimiter: '>'
Field separator: ','

COMMANDS:
---------

1. CONNECTION HANDSHAKE
   Request:  <C>
   Response: <C,OK>
   Purpose:  Initialize/verify connection and attach all servos

2. DISCONNECTION HANDSHAKE
   Request:  <D>
   Response: <D,OK>
   Purpose:  Detach all servos and close connection

3. ENABLE TORQUE (Supports multiple motors)
   Request:  <E,motor_id1,motor_id2,...>
   Response: <E,motor_id1,OK,motor_id2,OK,...>
             <E,motor_id,KO> (if motor_id out of range [0,5])
   Purpose:  Enable torque on specified motor(s)

4. DISABLE TORQUE (Supports multiple motors)
   Request:  <T,motor_id1,motor_id2,...>
   Response: <T,motor_id1,OK,motor_id2,OK,...>
             <T,motor_id,KO> (if motor_id out of range [0,5])
   Purpose:  Disable torque on specified motor(s)

5. WRITE SERVO POSITION (Supports multiple motors)
   Request:  <W,motor_id1,value1,motor_id2,value2,...>
   Response: <W,motor_id1,value1,motor_id2,value2,...>
             <W,motor_id,KO> (if motor_id out of range or no value provided)
   Purpose:  Set target position for servo(s). Values clamped to [servoMinCmd, servoMaxCmd]
   Notes:    Values subject to PID control depending on controller mode (PID or direct)

6. READ SERVO COMMAND POSITION (Supports multiple motors)
   Request:  <A,motor_id1,motor_id2,...>
   Response: <A,motor_id1,position1,motor_id2,position2,...>
             <A,motor_id,KO> (if motor_id out of range)
   Purpose:  Read the last commanded position for servo(s)

7. READ FEEDBACK POSITION FROM POTENTIOMETER (Supports multiple motors)
   Request:  <F,motor_id1,motor_id2,...>
   Response: <F,motor_id1,feedback1,motor_id2,feedback2,...>
             <F,motor_id,KO> (if motor_id out of range)
   Purpose:  Read actual position feedback from potentiometer in degrees (0-180)

8. READ CURRENT (Supports multiple motors)
   Request:  <I,motor_id1,motor_id2,...>
   Response: <I,motor_id1,current1,motor_id2,current2,...>
             <I,motor_id,KO> (if motor_id out of range)
   Purpose:  Read current consumption in mA 

9. READ ALL TELEMETRY (Supports multiple motors)
   Request:  <R,motor_id1,motor_id2,...>
   Response: <R,(motor_id1,feedback1,current1,command_pos1),(motor_id2,feedback2,current2,command_pos2),...>
             <R,(motor_id,KO)> (if motor_id out of range)
   Purpose:  Read all telemetry: feedback position, current, and command position

10. CONFIGURATION REGISTERS  (Read/Write) FOR ALL SERVOS AT ONCE OR GENERAL VARIABLES (e.g. controller mode or controller mode)
   Available Registers: STP, MINCMD, MAXCMD, MINFB, MAXFB, KP, KD, KI, CMOD, BAUDRATE

   A. SERVO START POSITION (STP)
      Read:   <R,STP> → <R,STP,pos0,pos1,pos2,pos3,pos4,pos5>
      Write:  <W,STP,pos0,pos1,pos2,pos3,pos4,pos5> → <W,STP,pos0,pos1,pos2,pos3,pos4,pos5>

   B. MIN COMMAND (MINCMD)
      Read:   <R,MINCMD> → <R,MINCMD,min0,min1,min2,min3,min4,min5>
      Write:  <W,MINCMD,min0,min1,min2,min3,min4,min5> → <W,MINCMD,min0,min1,min2,min3,min4,min5>

   C. MAX COMMAND (MAXCMD)
      Read:   <R,MAXCMD> → <R,MAXCMD,max0,max1,max2,max3,max4,max5>
      Write:  <W,MAXCMD,max0,max1,max2,max3,max4,max5> → <W,MAXCMD,max0,max1,max2,max3,max4,max5>

   D. MIN FEEDBACK (MINFB)
      Read:   <R,MINFB> → <R,MINFB,minfb0,minfb1,minfb2,minfb3,minfb4,minfb5>
      Write:  <W,MINFB,minfb0,minfb1,minfb2,minfb3,minfb4,minfb5> → <W,MINFB,minfb0,minfb1,minfb2,minfb3,minfb4,minfb5>

   E. MAX FEEDBACK (MAXFB)
      Read:   <R,MAXFB> → <R,MAXFB,maxfb0,maxfb1,maxfb2,maxfb3,maxfb4,maxfb5>
      Write:  <W,MAXFB,maxfb0,maxfb1,maxfb2,maxfb3,maxfb4,maxfb5> → <W,MAXFB,maxfb0,maxfb1,maxfb2,maxfb3,maxfb4,maxfb5>

   F. PROPORTIONAL GAIN (KP)
      Read:   <R,KP> → <R,KP,kp0,kp1,kp2,kp3,kp4,kp5>
      Write:  <W,KP,kp0,kp1,kp2,kp3,kp4,kp5> → <W,KP,kp0,kp1,kp2,kp3,kp4,kp5>

   G. DERIVATIVE GAIN (KD)
      Read:   <R,KD> → <R,KD,kd0,kd1,kd2,kd3,kd4,kd5>
      Write:  <W,KD,kd0,kd1,kd2,kd3,kd4,kd5> → <W,KD,kd0,kd1,kd2,kd3,kd4,kd5>

   H. INTEGRAL GAIN (KI)
      Read:   <R,KI> → <R,KI,ki0,ki1,ki2,ki3,ki4,ki5>
      Write:  <W,KI,ki0,ki1,ki2,ki3,ki4,ki5> → <W,KI,ki0,ki1,ki2,ki3,ki4,ki5>

   I. CONTROLLER MODE (CMOD)
      Read:   <R,CMOD> → <R,CMOD,mode>
      Write:  <W,CMOD,mode> → <W,CMOD,mode>
      Values: 'OFF' or 'PIDPOS'

   J. BAUDRATE (BAUDRATE - read-only)
      Read:   <R,BAUDRATE> → <R,BAUDRATE,9600>
      Write:   <W,BAUDRATE> → <W,BAUDRATE,9600>

CONTROLLER MODES:
-----------------
- 'OFF':     Direct servo control via servo controller (proportional)
- 'PIDPOS':  PID-based position control with continuous feedback adjustment

MOTOR PARAMETERS (Configurable in Arduino code):
-------------------------------------------------
- servoMinCmd/servoMaxCmd:   Command position limits (typically 0-180°)
- servoMinFeedback/servoMaxFeedback: Potentiometer calibration range (typically 0-1023)
- KP, KI, KD:  PID tuning constants per motor
- servoStartPos: Initial position on startup

ERROR HANDLING:
---------------
- Invalid motor_id (outside [0,5]): Returns <command,motor_id,KO>
- Missing arguments: Command may be ignored or partially processed
- Unknown command: Silently ignored

EXAMPLES:
---------
Connect:                  <C> → <C,OK>
Set motor 0 to 90°:       <W,0,90> → <W,0,90>
Read motors 0 & 1 target position: <A,0,1> → <A,0,90,1,85>
Disable motors 0,2:       <T,0,2> → <T,0,OK,2,OK>
Read telemetry motor 0:        <R,0> → <R,(0,512,150,90)>
Read start positions:     <R,STP> → <R,STP,100,170,100,100,90,90>
Write KP gains:           <W,KP,10,20,15,10,45,10> → <W,KP,10,20,15,10,45,10>
Read controller mode:     <R,CMOD> → <R,CMOD,PIDPOS>
Write controller mode:    <W,CMOD,OFF> → <W,CMOD,OFF>
"""


NB_SERVOS=6

class SG90MotorsBus(MotorsBusBase):
    START_MARKER = ord('<')
    END_MARKER = ord('>')

    BAUDRATE=9600

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
        if data_name not in ["F", "I", "A"]:
            raise ValueError(f"""Unsupported data_name for read: {data_name}. R command unsupported to do LeRobot typing interface
                              (forces int|float not list[int|float] ). Supported: 'F', 'I', 'A'""")
        msg = f"<{data_name}, {motor}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<{data_name}, {motor}, KO>":
            raise ValueError(f"Failed to read value for motor {motor}")
        return response

    def write(self, data_name: str, motor: str, value: Value) -> None:
        """Write a value to a single motor."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        if data_name != "W":
            raise ValueError(f"Unsupported data_name for write: {data_name}. Only 'W' supported for single motor write.")
        msg = f"<{data_name}, {motor}, {value}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if response == f"<{data_name}, {motor}, KO>":
            raise ValueError(f"Failed to write value for motor {motor}")
        return response
        

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, Value]:
        """Read a value from multiple motors."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        if data_name not in ["F", "I", "A"]:
            raise ValueError(f"""Unsupported data_name for sync_read: {data_name}. R command unsupported to do LeRobot typing interface
                              (forces dict[str, int|float] not dict[str, list[int|float]] ). Supported: 'F', 'I', 'A'""")
        msg = f"<{data_name}, {",".join(motors) if isinstance(motors, list) else motors}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        parse_response = response.split(',').replace(f'<{data_name},', '').replace('>', '').strip().split(',')
        if 'KO' in parse_response:
            raise ValueError(f"Failed to read value for motor {parse_response.index('KO')-1}")
        return {x.split(",")[0] : int(x.split(",")[1]) for x in  re.findall("[^,]+,[^,]+",response.replace(f"<{data_name},","").replace(">",""))}
        

    def sync_write(self, data_name: str, values: dict[str, Value]) -> None:
        """Write values to multiple motors."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        if data_name not in ["W"]:
            raise ValueError(f"Unsupported data_name for sync_write: {data_name}. Supported: 'W'")
        msg = f"<{data_name}, {','.join( f"{x} , {values[x]}" for x in sorted(values.keys()))}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        parse_response = response.split(',').replace(f'<{data_name},', '').replace('>', '').strip().split(',')
        if 'KO' in parse_response:
            raise ValueError(f"Failed to write value for motor {[parse_response[i-1] for i, x in enumerate(parse_response) if x == 'KO']}")
        return {x.split(",")[0] : int(x.split(",")[1]) for x in  re.findall("[^,]+,[^,]+",response.replace(f"<{data_name},","").replace(">",""))}



    def enable_torque(self, motors: dict[str, bool] | None = None, num_retry: int = 0) -> None:
        """Enable torque on selected motors."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<E, {','.join( f"{x} , {motors[x]}" for x in sorted(motors.keys()))}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        parse_response = response.split(',').replace('<E,', '').replace('>', '').strip().split(',')
        if 'KO' in parse_response:
            raise ValueError(f"Failed to write value for motor {[parse_response[i-1] for i, x in enumerate(parse_response) if x == 'KO']}")
        return response

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        """Disable torque on selected motors."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<D, {','.join( f"{x} , {motors[x]}" for x in sorted(motors.keys()))}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        parse_response = response.split(',').replace('<D,', '').replace('>', '').strip().split(',')
        if 'KO' in parse_response:
            raise ValueError(f"Failed to write value for motor {[parse_response[i-1] for i, x in enumerate(parse_response) if x == 'KO']}")
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

    # ========== CONFIGURATION REGISTER METHODS ==========

    def read_register(self, register: str) -> list:
        """Helper function to read configuration register values. Returns list for array registers, or raw value for scalar registers."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        msg = f"<R,{register}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        # Parse response: "<R,STP,val0,val1,val2,val3,val4,val5>" or "<R,CMOD,PIDPOS>" or "<R,BAUDRATE,9600>"
        parts = response.split(',')
        if len(parts) > 2:
            # Check if it's numeric array or string value
            try:
                return [int(v) for v in parts[2:] if v]
            except ValueError:
                # String value (for CMOD)
                return parts[2].strip()
        return []

    def write_register(self, register: str, values) -> None:
        """Helper function to write configuration register values. Accepts list or string."""
        if not self.is_connected:
            raise ConnectionError("Not connected to motors.")
        if isinstance(values, list):
            values_str = ','.join(map(str, values))
        else:
            values_str = str(values)
        if register == "CMOD" and values not in ["OFF", "PIDPOS"]:
            raise ValueError(f"Invalid CMOD value: {values}. Must be 'OFF' or 'PIDPOS'")    
        msg = f"<W,{register},{values_str}>"
        self.sendToArduino(msg)
        while self.serial.in_waiting == 0:
            pass
        response = self.recvFromArduino()
        if "KO" in response:
            raise ValueError(f"Failed to write register {register}")
 
  


