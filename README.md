# phantom_omni
Phantom Omni Manipulator Training for SRD laboratory  
Contact: Tomoki Takekawa (t.takekawa@srd.mech.tohoku.ac.jp)  

## Instruction

1. Install Ubuntu Xenial 16.04 (Dual-boot / VMWare)

2. Install ROS Kinetic  
Please refer to http://wiki.ros.org/kinetic/Installation/Ubuntu  
On Terminal
```
sudo apt-get update  
sudo apt-get install ros-kinetic-desktop-full
```

3. Make your workspace  
Please refer to http://wiki.ros.org/catkin/Tutorials/create_a_workspace
On Terminal
```
cd
mkdir catkin_ws  
cd catkin_ws  
mkdir src  
catkin_make  
```
or  
```
catkin_build  
```
(catkin build is a package for building, making, and organizing catkin workspace which provides more option than the ROS-built-in catkin_make. In case you are interested in using catkin build, please refer to https://catkin-tools.readthedocs.io/en/latest/installing.html)  

Source your workspace  
```
nano ~/.bashrc  
```
Go to the bottom line, type  
```
source ~/catkin_ws/devel/setup.bash  
```
Ctrl+X to exit, Press Y then Enter to save

4. Clone this repository  
```
cd ~/catkin_ws/src  
sudo apt-get install git  
git clone https://github.com/shaketomo/phantom_omni
```

5. Install dependencies and libraries  
```
cd catkin_ws/  
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y  
sudo apt-get install libncurses5-dev libncursesw5-dev
```

6. Compile the code  
```
catkin_make
```
or
```
catkin build
```

7. Debugging
Debug the sample code (omni_angle/src/main.cpp). The rest of packages/codes can be left without any changes.
Once you have debugged the sample code, try
```
roslaunch phantom_omni omni.launch  
rosrun omni_angle omni_angle  
```

The phantom omni should move all the joints 30 degrees.
