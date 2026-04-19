WORK IN PROGRESS:TODO (in progression order)
- add mechanical building indications
- add control layer to firmware (impedance control)
- integration with LeRobot framework
- first tests and demos

# LeMeccaBot

This repo present a homemade 4-dof arm with a gripper currently under development. The aim is to propose an robot arm cheaper and compatible with Hugging Face SO100 or S0101 for intiation to LeRobot framework. 
For cost reduction, 9g servos are used. Quite selfishly, supports are made with what I have: Meccano pieces.   

<div><img src="images/general_view.jpeg" style="width:50%" onerror="this.onerror=null; this.src='images/general_view.jpeg';"/> </div> 

## Hardware

For fast shipping (and as I'm not an experienced CAD designer), so I haven't done any 3D modelling or real-time physics simulation, but rather some simple calculus to size parts and motors.

That said, you'll find all pieces references and sizes in ```hardware/hardware_list.md``` as well as associated quantities. 

As the reduced price is the main purpose of this work, there is some modifications to do on servos in order to monitor position. The process is describe in ```hardware/motor_modification.md``` wich ends with 4-pins PWM servos. 

Note that compared to any serious robotic servo, only position and current are monitored, not temperature. 

To build the robot itself, find the building guide here ```hardware/building_guide.md```.

## Circuit

Circuit is kept as simple as possible, considering Arduino Uno R3 ports and limitations. In further optimisation, I may change controller to a cheaper and more apropriated one. Simply, Arduino Uno is one of the most rife microcontroller, so you may have one already if you plan to do this robot. It was the case for me. ;)

See [here][./hardware/electronical_circuit.md ] for the scheme. 

NB: A proper IC specific design might come in the future, but for now, let's make things work first. :)

## Firmware

Since the servos used are working with PWM and considering position feedback modification, it implies to develop a specific firmware. Once again, for cost reduction, instead of having a serial protocol and its motor drivers, I've choosen a parallel protocol to get only one controller for the whole arm.

The Firmware allows to monitor position and torque (current) of each motor (and not temperature). Simply flash ```./firmware/sg90_arduino/sg90_arduino.ino```on the controller  and use sg90.py on computer side. 
It contains a homemade protocol that allow to read and write concurrently on several motors as well as all base handshakes and enabling commands.  


## LeRobot classes

In progress


