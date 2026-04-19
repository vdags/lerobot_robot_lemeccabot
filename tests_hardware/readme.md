This folder will contain simplier circuits to test your robot or spare parts step by step while building. 
Each script is alongside its circuit scheme and eventually a tutorial's link for further information.

- ```motor_test``` contains a simple script for testing your motors out of the box before doing any manipulation on them by moving them from 0° to 180° continuously.

- ```motor_analogpos_test``` is for testing a motor after having modified it (adding potentiometer return). As ```motor_test``` it spans continuously from 0° to 180° but returning also the value of the potentiometer. If you are using Arduino IDE, use SerialMonitor for plotting position feedback (unmapped to 0°-180°, since its boundary constants vary from a motor to an other).

- ```motor_pos&torque``` is an extra circuit for educational purpose with is for testing circuit with position (motor's potentiometer) and torque feedback. This circuit is the base on with final circuit is made. If you are using Arduino IDE, use SerialMonitor for plotting position feedback (unmapped to 0°-180°, since its boundary constants vary from a motor to an other) and torque feedback (unmapped too).

- ```rotary_encoder_control``` is an extra circuit for controlling the robot directly through rotary encoder (one for each motor). This requires 5 rotary encoders (not listed as required pieces) but allows to do a hardware check. (I've used it for controlling that motors' torques were strong enough). This is not necessary for building process.
