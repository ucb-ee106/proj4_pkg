Welcome to Project 4 - Safe & Decentralized Multi-Agent Control!

In case you forgot how to run a Turtlebot, here are the steps below:

1. Place the Turtlebot on the ground and turn it on.
2. In your .bashrc, make sure the block for Turtlebot is uncommented and the lines corresponding to the workstation is the IP address.
3. Open a terminal and run 'roscore'
4. Open another terminal and ssh into the robot: ssh fruitname@fruitname where 'fruitname' is the name of the Turtlebot
4. Password: fruitname2022
5. In the ssh'ed terminal, run: roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen
6. To test that you can move the Turtlebot: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
7. To get the laserscan topic to show up, run: roslaunch turtlebot3_bringup turtlebot3_remote.launch