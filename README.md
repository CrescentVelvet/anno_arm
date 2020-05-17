# anno_arm
##配置环境 
```sh
    $ ROS_VERSION=`/usr/bin/rosversion -d`   
    $ sudo apt-get install ros-melodic-moveit-*   
    $ sudo apt-get install ros-melodic-industrial-*   
    $ sudo apt-get install ros-melodic-gazebo-ros-control   
    $ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers   
    $ sudo apt-get install ros-$melodic-trac-ik-kinematics-plugin   
    $ sudo apt-get install ros-melodic-usb-cam   
``` 
- Set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).
- Clone the repository into the src/ folder of workspace   
- Use "catkin_make" to build workspace
- Copy probot_rviz_plugin/plugin/libprobot_rviz_plugin.so to 'WORKSPACE_PATH'/devel/lib
- Set up environment variables:   
```sh
$ echo "source ~/'WORKSPACE_PATH'/install/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```


##安装OpenCV 


