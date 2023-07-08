# Antismiler
 Arduino-controlled robot that jet-sprays liquids at un-smiling faces

A full tutorial is available on my [blog post](https://medium.com/@idanmalka2001/smile-detection-water-gun-antismiler-destroyer-293840b6b13 "Medium Blog"), Please check it out!

### System Mechanism Overview
A camera feed is processed with OpenCV on the laptop to detect face position, and the smile. The tip of a tube is constantly directed at the person using a 2-degree-of-freedom servo-controlled manipulator. To load the spray gun, a peristaltic pump is activated while a servo-controlled Valve is closed to increase the pressure. If no smile is detected for 10 seconds, The valve is opened to release water from the needle tip.

--- 

## Hardware Requirements:
- Microcontroller with IO pins (Arduino, ESP...)
- 3x Servo Motors (Any)
- 1x Peristaltic Pump
- Tubing and needle tip

## Software Requirements:
- Install Python version >3.8
(if not, modify match case to if else statements)
- Install Pyserial, Open-CV:

`py -m pip install opencv-python`

`py -m pip install pyserial`

---

## Serial Communication
Serial Communication
The serial communication is done through a USB port, which can be opened from Python with the Pyserial library. We must now choose a template for each command to send to the Arduino as well as an acknowledgement package returned from the Arduino to prevent overflowing it with commands. We can start and end each package with square brackets to separate them. There is one robot arm with two servo motors, one peristaltic pump, and one servo that controls a valve.

The template package for the arm will include angles for both servomotors directly to save time. The operator will be the letter A for Arm, and the angles for the two servos will be written in order after S for state. A similar process for pumps and valves can be applied:


### Example Commands sent
| Hardware Action    | Command       | 
|:------------------|:-------------:|
| Pump 5 ml     | [P1 m5.00] | 
| Robot Arm (120°, 80°)   | [A1 X120 Y80]  | 
| Open Valve | [V1 S1]   | 

### Responses
| Command   | Description       | 
|:------------------:|:-------------|
| [ACK]  | command was received successfully | 
| [ERR] | There is an error in the command received | 
| [FREE] | The command has been processed and arduino is ready for the next  | 

Responses: [ACK], [ERR], [FREE]

The code is inspired from the communication protocol from my previous project, written by J. Bettles [SOURCE](https://github.com/UoM-team5/comms "Comms")

