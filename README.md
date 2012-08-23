## Contents
1. Prerequisites
2. Getting the software
3. Building
4. Running


## Prerequisites
Before installing the system you will need two things installed:
* Ubuntu Linux (11.10)
* ROS Electric

## Getting the Software
The easiest way to get the software is to clone a copy from GitHub using:  
````git clone git@github.com:rfw/se306p1.git````  
Navigate into the project directory using `cd se306p1`. You will now have a copy of all the code that is ready to be built.  

## Building
To build the project, run ````./build.sh```` in a terminal from the se306p1 directory. This will build all the code and run all the tests. In the process of building it will start roscore for use by the testing system. The build will output a message indicating success or failure, then stop roscore.  

## Running
To run the system 2 things must be loaded in separate terminals:  
Terminal 1: ````roscore````  
Terminal 2: ````./launch.py````  
The launch script takes a number of arguments allowing you to configure the number of robots, the number of robot groups, and the distance of the robots from the origin. For usage information about the launch script, run:  
````./launch.py -h````