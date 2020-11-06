# Turtle bartender app in ROS #

This repository hosts the source code for the ROS < ros_turtle_bartender > package which setups a turtle party and a bartender who takes drink orders from a webserver and delivers drinks to happy customers.

I created this game to practice using the ROS TF2 funcitionality.  The tf2 maintains the relationship between coordinate frames in a tree structure buffered in time, and allowed the bartender to find where the `drink stands` are located to pick up the correct drink and where the `customers` where located to deliver the drinks.

## Key concepts covered ##
- The goal of this app is to practice the TF funcitionality of ROS.
- As part of the app I wanted the work to be distributed from a webserver to simulate how a robot in the wild might be assigned tasks.
- The app will be to spawn a number of customers who are moving around randomly in a turtlesim space.
- One turtle will be a bartender who gets a drink order from a web server and must deliver to a specific turtle customer
- There are 4 drink options: Red Wine, White Wine, Vodka and Jamison.  Depending on which drink the bartender will need to move to a designated corner of the screen to pick up the drink then deliver the drink to the customer to simulate an automated drink stand.
- The TF broadcasting functionality will be used to for the bartender to find the way to the customer who is actually always moving (or dancing) in the area
- Once the drink is delivered the turtle bartender will ask the webserver for the next drink order
- The webserver will simply randomly choose the drink and customer to deliver.


![image info](./pictures/HideandSeek.gif)

Here on YouTube I quickly go through running the game and the code.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/a16sIjR4Cco/0.jpg)](http://www.youtube.com/watch?v=a16sIjR4Cco "Learning ROS through programming - Turtlesim Hide and Seek")

## Usage ## 

To use the `ros_turtle_bartender` package clone this repository into the `src` folder of your catkin workspace.

Then build the workspace with `catkin_make`.

In the `ros_turtle_bartender` directory using the `terminal` enter the `webserver` directory.

Run `node init` command to grab all of the node_modules required.  If you do not have nodjs install you can do so by using the command:

`# Using Ubuntu
curl -sL https://deb.nodesource.com/setup_15.x | sudo -E bash -
sudo apt-get install -y nodejs`

Finally start the package using roslaunch command: `roslaunch ros_hideandseek hideandseek.launch`

To change the starting location of the ball and search density edit the hideandseek.launch file using the following parameters:
- `<param name="/object_origin_x" type="double" value="5.0" />`
- `<param name="/object_origin_y" type="double" value="8.0" />`
- `<param name="/search_step_size" type="double" value="0.5" />`

## Node descriptions ##

Below is a picture of the rqt_graph outlining the nodes and the topics they use for communication.  Here is a brief description of each node moving from left to right in the rqt_graph.

### /next_goal_server ###
- this node is the server for the NextGoal service which calculates the next goal for the turtlesim robot to drive towards as it searches for the ball.  When the next goal is outside of the turtlesim environment it sends a complete message to stop the game.

### /turtlesim_node ###
- this is the main node for the turtlesim robot.

### /turtle_mark_target ###
- this node setups the playing field by drawing a circle at the x,y location specified in the roslaunch file.  When the setup is complete it publishes on the /start_search topic the search can begin.

### /turtle_color_search ###
- this node subscribes to the /turtle1/color_sensor topic and looks for the r,g,b values to be above 200 signalling it has found the ball.  When the ball is found it will publish a message on the /status topic to signal the search is complete.

### /turtle_move ###
- this node is what gives velocity commands to move the turtlesim robot.  When it reachs a goal it will request the next goal from the next_goal_server until it receives a complete from the server or a complete from the /turtule_color_search.
