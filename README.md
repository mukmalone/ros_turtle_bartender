# Turtle bartender app in ROS #

- The goal of this app is to practice the TF funcitionality of ROS and to ask for a job to perform from a webserver.
- The app will be to spawn a number of customers who are moving around randomly in a turtlesim space.
- One turtle will be a bartender who gets a drink order from a web server and must deliver to a specific turtle customer
- There will be 4 drink options.  Depending on which drink the bartender will need to move to a designated corner of the screen to pick up the drink then deliver the drink to the customer
- The TF broadcasting functionality will be used to for the bartender to find the way to the customer who is actually always moving (or dancing) in the area
- Once the drink is delivered the turtle bartender will ask the webserver for the next drink order
- in the webserver it will somehow randomly choose the drink and customer to deliver
