# Merlin Roller
Development and testing were all done using Microsoft Visual Studio Code. 
Stack Overflow was used extensively to help troubleshoot compiler and linker 
errors, as well as for tips on how to create and build using CMakeLists.txt. 
Alex, YouTube and various lecture materials online proved handy for brushing up 
on the physics pertinent to this problem. No other software tools or aids were 
consulted in the process.

The instructions for building and running the program are outlined below.

## Building
Unzip the file and locate the `MerlinRoller` directory in a terminal window.
To build with CMake, run the following commands.

```
cd build
cmake ..
make
```

## Running
To run the executable you have just built, run

```
./MerlinRoller
```

## Limitations
This solution handles the case where a ball impacts one slope and then hops onto
another one. However, it does not account for any ball-ball collisions (i.e.
collisions are only with slope object surfaces). It also does not simulate
bouncing or partial energy dissipation. Instead, the vast majority of kinetic
energy from dropping the ball from some height above the surface is
instantaneously cancelled by the normal force.

As the simulation step size `dt` decreases and nears zero, the closer it 
approximates continuous time. Taking the example of simulating a ball's motion 
with some initial x velocity on a flat surface with `dt = 0.02` and `dt = 0.5`
(see Ball 2 defined in main.cpp), one can see that the time it takes for its
surface speed to catch up to its linear speed can be very different. Using 
`dt = 0.02`, it takes the ball roughly 0.32 seconds for the speeds to become 
equal, over which it has traveled only 0.7664 meters. With `dt = 0.5` it takes
4.5 seconds, over which it has traveled 9.52 meters! By using a larger step
size, it is easy to overshoot the parameters in the integration. In this case,
it took a much longer time for the surface speed and linear speed to converge.
By taking smaller steps, one can more accurately simulate the real time dynamics
for this system without overshooting.