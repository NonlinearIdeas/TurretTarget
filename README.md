TurretTarget
============

Implements and simulates an algorithm for a rotating turret to launch a projectile that strikes a target. 

The algorithm for hitting a moving target with constant velocity is relatively simple when the shooter does not have to rotate.
When it does, the math gest much more complicated.  

One solution is to continually rotate towards a position you think the 
target will cross, each time calculating if you can make the shot.

Another solution, this one, solves the equations of the situation by "squeezing" the solution time between an upper and lower
limit.  

This code base contains the algorithm implementation as a simple function, along with a simulation of the target moving and
the shooter for experimentation.
