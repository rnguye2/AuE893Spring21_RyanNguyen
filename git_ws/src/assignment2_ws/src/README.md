The three scripts do the following:

Circle.py: creates a circular trajectory for a TurtleBot and presets a velocity.  From the launch file you can tune the radius to your liking.

![Alt text](circle.png?raw=true "Circle")

square_openloop.py: creates a square trajectory for a TurtleBot where the linear and angular velocities are preset to 0.2 m/s and 0.2 rad/s, respectively. The TurtleBot can travel the trajectory for as long as the desired time.

![Alt text](square_open.png?raw=true "Open Loop square")

square_closedloop.py: creates a square trajectory for a TurtleBot where the linear and angular velocities are controlled. My control method had the TurtleBot slowing down towards each corner of the sqare. It would start out fast, reduce speed and then come to a full stop before rotating. It performed this action on each leg of the square and it terminates when the square is completed.

![Alt text](square_closed.png?raw=true "Closed Loop square")



