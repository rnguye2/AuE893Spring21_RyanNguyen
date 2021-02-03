In move.lauch there are 2 python scripts that can be run. 

The first script is circle.py which generates a turtlebot that travels a circular path.

The second script is square.py which generates a turtlebot that travels a square path.

The turtlebots are then deployed in an empty world map to perform their designated actions. To run move.launch use the follwing in a terminal:

Circle.py: $ roslaunch assignment3_turtlebot3 move.launch code:=circle
Square.py: $ roslaunch assignment3_turtlebot3 move.launch code:=square




In emergency_braking.world one script can be run: emergency_braking.py

This launch file generates a world with a singular wall that the turtlebot has to identify and then brake before making contact. the script has the turtlebot traveling in a striaght line until it approaches the wall, when it passes the threshold it stops. To run emergency_braking.launch, use the following in the terminal:

emergency_braking.py: $ roslaunch assignment3_turtlebot3 emergency_braking.launch


