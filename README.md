Simulation Software
=================================

This repository contains the simulation code designed to work in conjunction with UWRT's main codebase [riptide_software](https://github.com/osu-uwrt/riptide_software.git). This repo is designed to work with the Gazebo simulator, which is part of the [Robot Operating System](http://www.ros.org/) framework.

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [RoboSub](https://www.auvsifoundation.org/competition/robosub) | [License](LICENSE)


![OSU UWRT Logo](https://github.com/osu-uwrt/riptide_software/blob/master/logos/UWRT_Logo_small.png)

# Building
ROS is compiled using the catkin build system, and so this repo will use catkin. 

## Cloning
To collaborate with the sim_software platform, you must fork this repo (click "Fork" at the top-right of this page). When executing the commands below, you will need to enter the URL to your forked repo. Form YOUR forked repo, click "Clone or download" at the top-right of the page, copy the URL, and then insert that URL in place of "<your_forked_repo>".
```
mkdir -p ~/osu-uwrt/sim_software/
cd ~/osu-uwrt/sim_software/
git clone <your_forked_repo> src
```

## Setting up Git Remotes
Since you just cloned your fork to your computer, your remote called "origin" will point to your fork. Now, create a new remote called "upstream" that points to this repo.
```
cd ~/osu-uwrt/sim_software/src/
git remote add upstream https://github.com/osu-uwrt/sim_software.git
```

Now, if you type:
```
git remote -v
```
You will see both a remote to your fork and to the main repo. You will use these two remotes a lot when pushing code to your fork, submitting pull-requests, and pulling down the latest code.

## Compiling
To compile this repo, you simply execute the "catkin_make" command from a terminal. As a word of caution, you MUST be inside the folder "~/osu-uwrt/sim_software" to run "catkin_make"
```
cd ~/osu-uwrt/sim_software/
catkin_make
```

## Chaining Workspaces
Please refer to the Readme for the [riptide_software](https://github.com/osu-uwrt/riptide_software.git) for how to chain workspaces.
