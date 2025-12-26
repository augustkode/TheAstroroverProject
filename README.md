THE ASTROROVER PROJECT

For full blog with pictures and videos:

https://dronesonen.usn.no/?p=13185

Abstract

This final blog sums up Sanders and Augusts work. The theme of this project is space. The task was wide, and provided us significant freedom of exploration and opportunities to grow within systems and computer engineering. From here we derived the problem domain and ultimate goal: safe autonomous data collection. To achieve this, we have used sensorics and actuators. Some of the sensors used are LiDAR, DHT11, Ultrasonic sesnsors, IMU and barometer. Some of the actuators used are mechanic wheels. The methods followed was kanban, agile scrum and UML. We achieved our ultimate goal of autonomous data collection. The data is displayed in a GUI, and the results are logged in a file. We collect data, both from the environment and from the system.
Preface
We would like to thank Steven Bos and Richard Thue for great help and guidance. We also want to thank Henning Gundersen for great music.
Demo Video
Table of Contents

    Problem Domain
    System Requirements
    System Design
    Challenges
    Improvements
    Conclusion

Problem Domain:

The problem domain of our project are safe autonomous data collection. We are of course not on mars, so our concerns lies in how to make the car autonomous, how to gather sensor data and how to display the sensor data.
Our greatest challenge have been merging every working and verified sub-system, and make them collaborate in real-time.
Our starting point was a pre built schaled down electric car, with mechanum wheels. The challenges and concerns raised was:

    How could we make the car drive fully autonomous?
    What data should we collect?
    How could we gather data?
    How could we mount the sensors?
    How could we display the data?
    How could we control the car incase of an emergency?
    How could we see where to drive?
    How to fit and place all the components within a small frame?
    How can we map the environment?
    How to fit everything?

Tools

We have held one group meeting a week, where we have talked about progress. We have used kanban and agile scrum. Most of the work have been done at home.

Development Tools:

    tree.aiga
    Github
    ROS2 humble
    Ubuntu 22.04
    Ubuntu 22.04 Server (RPI)
    VSCode (with remote SSH)
    RViz 2
    Gazebo
    DrawIO
    Qt

System Requirements

We split the system into three requirement categories:

    System requirements
    Rover requirements
    GUI requirements

System Design
The system
The system scoped down
ROS2 Diagram
Displaying all the nodes and topics in our system
Features
Raspberry PI 4B

Runs:

    Lidar_node (stl lidar node, provides lidar input to /scan)
    Autonomous_node (Autonomous driving and lidar processing, using lidar data)
    motor_driver node (Parsing driving commands to micro:bit from cmd_vel and cmd_vel_auto)
    Arduino_bridge_node (Provides functionality. DHT11, LCD, PIR, RGB, Photoresistor)
    temperature_tracker node (Provides PI CPU temperature)
    camera_node (provides camera stream from ZeroCam NightVision)
    sensehat_node (Privides IMU data, barometer, and car internal temperature)

Micro:Bit V2
Runs:

    Parses driving commands from motor_driver mode to run motors.
    Ultrasonic sensors (Emergency stop, separate from ROS 2, to protect camera in front in case of user error or autonomous malfunction)
    Motor Driver Board (Controls motors)

Arduino Mega 2560
Runs:

    DHT11
    LCD display
    Photoresistor
    PIR sensor
    RGB light
    Buzzer

PC
Runs:

    GUI node (displays live camera feed, system status and other sensor information)
    Teleop_twist node (Processes /joy data and publishes on /cmd_vel)
    Joy_node (Handles joystick input and publishes to /joy)

Component List
Car Design
The car design. Displaying the frame with its placeholders.

Challenges

We have met two great challenge, that we have yet to defeat. We have not been able to make mapping possible with IMU data, and we are experiencing intermittent faults in our arduino sub-system.
We get LiDAR visualization in Rviz, and we can somehow measure and display the room. But we have not been able to integrate the IMU data with the LiDAR.
The intermittent faults we are experiencing comes and goes. Most of the time, the lcd display and DHT11 sensor works. These are the most important components from the arduino, so this is fine. We have tried to solve the problem by removing every unessecary jumer cable, but this didnt solve the problem.

Improvements

If we where to do this again, we would have used wheel encoders to get odometri combined with LiDAR for more accurate mapping. We would also buy a powerbank without PD. These have to much constraints, and wouldnt drive the motor board. We would also used kanban on Github instead of tree.aiga, because everything would have been on the same site.
Conclusion
We achieved our ultimate goal, safe autonomous data collection. The system is also able to map the environment to a certain degree, check improvements for more information. The GUI, the joystick, and the vehicle are tightly interconnected. All three subsystems are essential for the overall system, and each one must function properly for the others to carry out their tasks.

Final words

We want to thank you for a great course! When we started out we felt kind of overwhelmed. We had a vision of what we wanted to do, but we where not sure how to do it. Now we know. The experience gained from this course can not be described. In january, when we are starting up writing our bachelor, we will continue on with Qt, VSCode and ROS2 on Ubuntu. The amount of hours we will save in january, already spent the hours on how this semester, is amazing.
Software might not solve every problem, but hours spent will.

This have been truly inspiring. We have genuinly matured as future engineers. Again, thank you!

– August & Sander.
