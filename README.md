# relaxed_ik_ros1

This is a cleaner version of RelaxedIK wrapped up in ROS1 with pre-generated config files of some mostly used robot arms. A keyboard pose goal driver and a rviz viewer are included in this wrapper for you to play with. However, you will not be able to customize your own config files in this wrapper. You can go to this link instead [relaxed_ik](https://github.com/uwgraphics/relaxed_ik) for the complete version. In the complete version, you will have the opportunity to (and have to) set up everything from scratch, including importing a urdf, configuring the collision files, and training the neural network, etc. The complete version is recommended if you would like to work on the configuration process yourself or a robot that we didn’t include in the available options; otherwise, this wrapper is likely to make your life much easier since everything you need to do will be setting the name of the robot you would like to work with.

## Dependencies
### Python Dependencies (Not optional)
kdl urdf parser:
```
sudo apt-get install ros-[your ros distro]-urdfdom-py
sudo apt-get install ros-[your ros distro]-kdl-parser-py
sudo apt-get install ros-[your ros distro]-kdl-conversions 
```

readchar:
```
sudo pip install readchar
```

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl
```
sudo pip install python-fcl
```

scikit learn:
http://scikit-learn.org/stable/index.html
```
sudo pip install scikit-learn
```

scipy:
```
sudo pip install scipy
```

Pyyaml:
```
sudo pip install PyYaml
```

Lastly, update your version of numpy:
```
sudo pip install --upgrade numpy
```

### Rust Dependencies (Not optional)
To use this wrapper, you will first need to install Rust.
https://www.rust-lang.org/learn/get-started

If you plan to extend any of the Rust code, we recommend using the Jetbrains rust plugin.

## Install
1. Install all the dependencies.
2. Clone this repo to the src directory in your ROS workspace.
3. Initialize relaxed_ik_core (The core part of relaxed IK written in Rust) as a submodule by running the following command in the project directory: 
	```
	git submodule update --init
	```

## Run
1. Configure the name of the pre-computed robot arm you would like to run with (available options are ur5, yumi, panda and iiwa7) in relaxed_ik_core/config/loaded_robot.

2. Compile relaxed_ik_core (The core part of relaxed IK written in Rust) by running the following command from the project directory:
	```
    cd ./relaxed_ik_core
	cargo build
    ```

3. Build your ROS workspace.
2. Run the following command to launch the solver: 
    ```
    roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
    ```

3. (Optional) Open a new terminal and run the following command to publish a new message to the topic EE pose goals to get the joint angle solutions (change the letter placeholders to actual numbers):
    ```
    rostopic pub -1 /relaxed_ik/ee_pose_goals relaxed_ik_ros1/EEPoseGoals '[0, now, base_link]' '[{position: [x, y, z], orientation: [x, y, z, w]}]'
    ```

4. View the robot arm in rviz by running the following command in a new terminal:
    ```
    roslaunch relaxed_ik_ros1 rviz_viewer.launch
    ```

5. In a new terminal, initialize the keyboard IK goal driver node to move the robot arm in rviz:
    ```
    rosrun relaxed_ik_ros1 keyboard_ikgoal_driver.py
    ```

6. To use the keyboard controller, please ensure that the termainal window where the keyboard_ikgoal_driver script was run from has focus (i.e., make sure it's clicked), then use the following keystrokes: 
	```
	c - kill the controller controller script
	w - move chain 1 along +X
	x - move chain 1 along -X
	a - move chain 1 along +Y
	d - move chain 1 along -Y
	q - move chain 1 along +Z
	z - move chain 1 along -Z
	1 - rotate chain 1 around +X
	2 - rotate chain 1 around -X
	3 - rotate chain 1 around +Y
	4 - rotate chain 1 around -Y
	5 - rotate chain 1 around +Z
	6 rotate chain 1 around -Z

	i - move chain 2 along +X
	m - move chain 2 along -X
	j - move chain 2 along +Y
	l - move chain 2 along -Y
	u - move chain 2 along +Z
	n - move chain 2 along -Z
	= - rotate chain 2 around +X
	- - rotate chain 2 around -X
	0 - rotate chain 2 around +Y
	9 - rotate chain 2 around -Y
	8 - rotate chain 2 around +Z
	7 - rotate chain 2 around -Z
	```
