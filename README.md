# DoitArm
Doit arm based on ESPDuino

An open source WiFi controlled 6 DOF robot arm on tank is designed in this project. The robot arm is driven by 6 digital servo. The tank is driven by 2 DC gear motors. All the digital servos and DC gear motors are driven by a 2 way motor & 16-way servo board. The controller is ESPDuino, which is designed from Doit team(www.doit.am). ESPDuino is an Arduino UNO R3 compatible ESP8266 development board based on ESP13 module.</br>
   </br>
The software is designed based on the project: https://github.com/esp8266/Arduino. A http webserver and UDP server are embedded. OTA function is integrated to update firmware freely. Robot inverse kinematics algorithm is implemented on ESPDuino. The end effector can be moved in tool or cartier coordinate system like a commercial robot arm! A simple UDP based command line interface is provided for controlling. </br>

Highlights:</br>
- 1, WiFi controlled 6-axis robot arm on Tank with ESPDuino.
- 2, ABB IRB4400 6-axis Industrial robots scaled model. Tank is T300 from Doit team.
- 3, Robot inverse kinematics algorithm running on ESP8266
- 4, A simple UDP based command line interface is provided.
- 5, Tank can be controlled via android APP. Robot arm can be controlled via a UDP client
- 6, OTA function is integrated to update firmware without usb cable.
- 7, Low cost for education, hobbies, hackers… 
- 8, Hardware and Software are Open Source.
