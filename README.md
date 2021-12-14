# Goubot

**Contributors:** Jeffery Zhou, Kenneth Tochihara, Charlie Ray

The goal of this project is to create a robot that can fetch a ball and bring it back home. "狗 (gŏu)" translates to "dog" in Chinese, which was the main naming convention for Goubot. Here is the [Youtube playlist with Project Update videos](https://youtube.com/playlist?list=PLUgYn1EdVdaukTL5irfwSSML7CPnV4CY7).

## Project Updates

### Project Update 1

Using the given Ubuntu image, `ECE470VM`, a virtual machine (VM) was set up on a computer with WMware Fusion. This was done to minimize the amount of overhead in setup and to quickly establish a working environment to interact with the robot. 

To test the simulation environment, modifications were made to the `lab2andDriver` package to create an initial enviroment of interacting with the UR3 robot. Using the content from lab 2, we applied principles of Robot Operating System (ROS) to verify connections between the Gazebo simulator and the controller. Nodes, publications, and subscriptions were estabilished to verify the movements of the joint and the gripping end-effector in our executions. 

Next steps will be to establish a unique robot with mobile capabilities and an gripping end effector tool. Furthermore, additional sensors will be added to find and interface with the desired ball. 

### Project Update 2

We were able to get the UR3 and the differential drive cart to work independently but not together. But upon further investigation, the cart did not properly respond to the commands sent to the drive when integrated. Gazebo physics made it extremely difficult for the wheels to spin and was not able to get the cart to move independently.

### Final Project Update

Because of the issues faced with the differential drive controller, a planar move controller was used instead to move the robot. We were able to get the robot to translate to the correct location, keeping in mind the motion planning logic and considering the task space of the robot. The final version of this project includes a randomly generated block that is picked at the spawn point and moved back to the original spawn point of the robot.

Though we were not able to get the robot to use the differential drive cart, there were still osme issues with the planar move robot. The rotation was not able to be induced. This prevented the cart of rotating at a reasonable rate, thus justifying the lack of orientation change. Furthermore, there was some error built up over time, that prevented the cart from moving to the correct location. This can be further improved with a better controller down the line.

## Acknowledgements

The team would like to acknowledge their instructors, Professor Katie Driggs-Campbell and Professor Mohamed Ali Belabbas, for the conceptual contributions. The team would like to also acknowledge their TA, Dhruv Mathur, for the application contributions with UR3. Finally, the team would like to acknowledge the iLuvRobotiks team, Karnap Patel, Ayush Nair, and Aditya Gupta, for their help throguhout the project.
