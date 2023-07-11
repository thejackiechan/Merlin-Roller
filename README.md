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
cmake..
make
```

## Running
To run the executable you have just built, run

```
./MerlinRoller
```

## Limitations
What limitations does your solution contain? If able, please comment on discrete-time errors relative to continuous time (i.e. real life) with varying discretization size. Notice anything odd? How would you characterize the error. Hint: Pay particular attention to how a ball starts rolling and describe that.