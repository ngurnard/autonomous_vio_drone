=======================================
MEAM 620 - Project 3
Nicholas Gurnard - ngurnard@seas.upenn.edu (personal - nk.gurnard@gmail.com)
=======================================

How to Run?
==========
The imports are structured to run from the directory .../projects/project_3/proj3/meam620/
The .../meam620/ folder can be considered the top level directory. Here there is a setup.py file which
installs all needed packages by running the command
        $ pip install -e .

The .../meam620/proj3/utils/ folder contains a few maps that can be used for testing the planner, controller and vio estimator.
Run the file .../meam620/proj3/utils/test.py to test all environments

The .../meam620/proj3/code/ directory contains a set of code files and sandbox files which constitute the main
planner, controller, and vio estimator code.
To see an animation and other useful plots, run the file .../meam620/proj3/code/sandbox.py
Modifying the filename variable will load different maps that exist in the util directory, which consists of .json files

Controller
==========

The controller implemented is a non-linear geometric PID controller found in se3_control.py. The best results were obtained with the following gains:

self.Kp = 0.6*np.array([[5, 0, 0],
                        [0, 5, 0],
                        [0, 0, 5]]) # proportional positional gain
self.Kd = 0.6*np.array([[3.5, 0, 0],
                        [0, 3.5, 0],
                        [0, 0, 3.5]]) # derivative positional gain
self.Kr = 0.5*np.array([[3750, 0, 0],
                        [0, 3750, 0],
                        [0, 0, 135]]) # proportional attitude gain
self.Kw = 0.5*np.array([[100, 0, 0],
                        [0, 100, 0],
                        [0, 0, 20]]) # derivative attitude gain

The controllers were coded following the project handout. The tuning was done manually, by checking small angle perturbations and position offsets (step responses).

Trajectories
============

The trajectory implemented with a minimum snap trajectory generator in world_traj.py.
Ray tracing was the main path pruning alogirhtm used, and time allocation was created as a non-linear cube-root function, i.e.:
    time_allotted = (segment_distance)^{1/3}/(avg_velocity)
The planner is an A* planner that is written under graph_search.py, which discretizes the map with occupancy_map.py

Results
=======

We can see that the results for the test cases in the utils directory are pretty good!
----------------------------------------------------------------------
pass window
    stopped at goal:        pass
    no collision:           pass
    flight time, seconds:   4.59
    flight dist, meters:    25.51
    planning time, seconds: 21.06
    exit message:           Success: End reached.

pass maze
    stopped at goal:        pass
    no collision:           pass
    flight time, seconds:   7.39
    flight dist, meters:    24.01
    planning time, seconds: 1.8
    exit message:           Success: End reached.

pass forest (custom map)
    stopped at goal:        pass
    no collision:           pass
    flight time, seconds:   7.13
    flight dist, meters:    22.56
    planning time, seconds: 3.61
    exit message:           Success: End reached.

pass over_under
    stopped at goal:        pass
    no collision:           pass
    flight time, seconds:   10.88
    flight dist, meters:    34.23
    planning time, seconds: 6.35
    exit message:           Success: End reached.



There were also hidden tests cases in the course autograder which were also successful, with the following results:

pass slalom
    stopped at goal:              pass
    no collision:                 pass
    flight time, seconds:         14.15
    flight dist, meters:          73.4
    flight time for max points:   20
    exit message:                 Success: End reached.

pass stairwell
    stopped at goal:              pass
    no collision:                 pass
    flight time, seconds:         12.21
    flight dist, meters:          34.69
    flight time for max points:   18
    exit message:                 Success: End reached.

pass switchback
    stopped at goal:              pass
    no collision:                 pass
    flight time, seconds:         11.69
    flight dist, meters:          76.16
    flight time for max points:   20
    exit message:                 Success: End reached.


The presented code resulted in the #2 best overall time in the class, with a total time of 60.9 seconds for all maps.
The switchback and window maps were the most successful trials, getting the #1 speed in the class (by more than a full second for switchback).

Collaborators
=============
Evan Grant and Orlando Lara gave the most insightful tips for this project. Namely, Orlando explained ray tracing to me
which was essential in getting my pruning algorithm to work (line-of-sight pruning or ray tracing). Evan was a great help
in discussing novel ways to get yaw control to work, however I never was able to get it successfully working (I have yet to figure out
how to handle yaw angle wrap around, and so it often does a 180 when yaw_control is set to True in world_traj.py).
He was able to, so if there are any questions you should as Evan Grant!


More Instructions on Getting to Run
===================================

Installation Guide

Prerequisites:

    PyCharm IDE Community Edition
        PyCharm is not explicitly required, however we strongly encourage using PyCharm. It’s a full featured IDE that offers powerful tools for debugging, managing virtual environments/ interpreters, and will be very familiar to those with a MATLAB background. I used VSCode.
    Python 3.8.X
        Linux: install it via package manager. Open Terminal and type the following commands:
            $ sudo apt-get install software-properties-common
            $ sudo add-apt-repository ppa:deadsnakes/ppa
            $ sudo apt-get update
            $ sudo apt-get install python3.8
        After the installation, check your Python 3.8 version with:
            $ python3.8 --version
        Windows: download installation package here (scroll down the page, use the recommended Windows installer option), also make sure to check the box “Add Python3.8 in PATH” when installing.
        Mac: download installation package here: https://www.python.org/downloads/release/python-387/  . Mac users need to run the “install Certificates.command” after the package is installed.
        
        
        
Getting Started:

    Download the .zip file from Canvas, which contains all the files related to the specific project, including the simulator (which is under /flightsim for future reference). It is important you do not move or replace any files unless directed to do so.

    Extract the .zip file into a folder of your choice.

    For the last step, you can either install using PyCharm or install using a terminal:

    Method 1 (install using Pycharm) [Highly Recommended]:
        Open PyCharm and create a new project (File>New Project)
        Set the “Location” to the path containing the setup.py file (../projX_Y/meam620/)
        Make sure to check “New environment using” and select “Virtualenv”, this should be the default. 
        Make sure the “Base interpreter” is Python 3.8
        All other settings should be unchecked, click "Create".

At this point, If you are prompted to "Create from Existing Sources" or "Open Project", click "Open Project".

        Once the project is created, the required packages have to be installed. Navigate to the Terminal at the bottom of the PyCharm window. In the terminal, type: 

pip install -e .

Note: if this line does not run successfully, you can still install the required packages manually, e.g. pip install numpy The required packages are found in setup.py

        Lastly, you must add a configuration which turns a python file into an executable that PyCharm can run–this can be done by clicking Add Configuration… at the top right. 

            Click the plus icon in the top left and select Python.

            Name it as you desire, and choose the script path leading to sandbox.py

At this point, you are ready to go! Refer to PyCharm documentation (or ask on Piazza) regarding usage instructions.

    Method 2 (Linux terminal):
        Open a terminal at the root of your project packet (the directory containing setup.py) and execute:

pip install -e .

After this step all dependencies will be installed. You may use whichever coding environment suits you.
