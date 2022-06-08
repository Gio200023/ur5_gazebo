<p align="center">
  <a href="">
    <img src="https://github.com/Gio200023/ur5_gazebo/blob/main/logo1.png">
  </a>
  <h2 align="center">UR5 and Gazebo</h2>

  <p align="center">
  Exam project for Fundamentals of Robotics
  <br>University of Trento 
  </p>
</p>
<br>

I made this prohect for Fundamentals of Robotics course. This is the setupping of the gazebo_ros world with the models needed.
Also in the spawner folder there is the spawner_2.py file which is a python script needed to spawn randomly lego blocks.

## How to run

Once you cloned the repository make sure on you computer ros noetic has been set. 
Create a catkin_workspace and put in the src folder the ur5_gazebo package.
Then you can proceed into the building of the package.
```
catkin build ur5_gazebo
source devel/setup.bash || source devel/setup.zsh #depends on your shell
```
> you need catkin_tools

Once the building phase has finished you can proceed into the launching of the world.
```
roslaunch ur5_gazebo BRICKS.launch
```