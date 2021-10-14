# Goubot

The goal of this project is to create a robot that can fetch a ball and bring it back home. "狗 (gŏu)" translates to "dog" in Chinese, which was the main naming convention for Goubot.

- [Google Drive with Project Update reports](https://drive.google.com/drive/folders/18sqLj46mc1D2UNInllISgHYMVMgnIE8k?usp=sharing)
- [Youtube playlist with Project Update videos](https://youtube.com/playlist?list=PLUgYn1EdVdaukTL5irfwSSML7CPnV4CY7)

### Project Update 1

Using the given Ubuntu image, `ECE470VM`, a virtual machine (VM) was set up on a computer with WMware Fusion. This was done to minimize the amount of overhead in setup and to quickly establish a working environment to interact with the robot. 

To test the simulation environment, modifications were made to the `lab2andDriver` package to create an initial enviroment of interacting with the UR3 robot. Using the content from lab 2, we applied principles of Robot Operating System (ROS) to verify connections between the Gazebo simulator and the controller. Nodes, publications, and subscriptions were estabilished to verify the movements of the joint and the gripping end-effector in our executions. 

Next steps will be to establish a unique robot with mobile capabilities and an gripping end effector tool. Furthermore, additional sensors will be added to find and interface with the desired ball. 
