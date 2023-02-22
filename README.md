
# Crash Avoidance Using Tello ROS Foxy

This project involves communication and control of four DJI Tello drones using ROS Foxy on a single network. The goal of this project is to coordinate the movement of multiple drones to accomplish a given task. The communication between the drones is established using Wi-Fi communication protocol.

In addition, the project implements a crash avoidance system that uses various sensors like camera, ultrasonic sensors, and LIDAR sensors to detect obstacles and avoid collisions. The crash avoidance system is implemented on each drone, allowing them to work autonomously while still maintaining coordination with the other drones in the network.

The project utilizes the capabilities of ROS Foxy, including the Publish-Subscribe model, Service-Client communication model, and the Actionlib library. The drones are controlled using the Publish-Subscribe model by publishing Twist messages to the drone's cmd_vel topic. The drones' takeoff and landing functions are called using Service-Client communication model. Additionally, the Actionlib library is used to coordinate the actions of the drones to accomplish specific tasks.

The project will involve the use of various algorithms, such as SLAM, PID controllers, and decision-making algorithms to achieve the objective of the project. It is an exciting project that demonstrates the potential of ROS and drone technology in solving real-world problems.

The project is ideal for those who are interested in robotics, computer vision, and control theory. It offers an opportunity to apply these concepts to a real-world project that involves multiple drones working together to achieve a common goal.

## Project Requirement

- python 3
- numpy
- dronekit
    
## Authors

- [@khulqu15](https://github.com/khulqu15)

Check my profile here [Mohammad Khusnul Khuluq](http://ninnoelka.ee.student.pens.ac.id/)


## License

[MIT](https://choosealicense.com/licenses/mit/)

