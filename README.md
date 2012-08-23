## Contents
1. Prerequisites
2. Getting the software
3. Build
4. Configure
5. Running


### Prerequisites:
Before installing the system you will need two things installed
* Ubuntu Linux (11.10)
* ROS electric

### Getting the software:
The easiest way to get the software is to clone a copy from github using  
````git clone git@github.com:rfw/se306p1.git````  
Then navigate into the project directory using cd se306p1  
You will now have a copy of all the code that is ready to be built.  

### Build:
To build the project, run ````./build.sh```` in a terminal.  
This will build all the code and run all the tests.   
In the process of build it will start roscore for use by the testing system.  
It will output if it was successful, then stop the roscore it started.  

### Configure:  
In the top folder there is a file called launch.config  
it should contain a single line with two integers separated by a space:  
````<number of robots in the system> <number of swarms to break into>````  
Configure the settings in this file and save.  

### Running:
To run the system 3 things must be loaded in separate terminals.  
Terminal 1: ````roscore````  
Terminal 2: ````./launch.py controller````  
Terminal 3: ````./launch.py supervisor````  


