### 配置环境
```sh
$ echo "source ~/'WORKSPACE_PATH'/install/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
### 运行机械臂
```sh
roslaunch probot_grasping probot_anno_grasping_demo.launch
```
### 运行视觉处理
```sh
roslaunch probot_grasping ibvs.launch
```
### 运行传送带
```sh
rosservice call /conveyor/control "state: power: 12.0"
```
### 运行小车
```键盘
q  w  e
a  s  d
z  x  c
```
### 效果图片

<img width=850 src="https://img-blog.csdnimg.cn/20201225152939422.png" alt="图像雅可比矩阵"/>


图像雅可比矩阵


<img width=850 src="https://img-blog.csdnimg.cn/20201225152954137.png" alt="小车随动抓取"/>


小车随动抓取


<img width=850 src="https://img-blog.csdnimg.cn/20201225153028877.png" alt="传送带随动抓取"/>


传送带随动抓取

### 卸载自带的OpenCV3.2.0
```sh
sudo apt-get purge libopencv* 
sudo apt autoremove
pkg-config opencv --modversion
```
查看opencv版本，查不到，成功卸载。
### 重新安装OpenCV4.2.0
```sh
sudo apt install  build-essential
sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt upgrade
sudo apt install libjasper1 libjasper-dev
```
其中 libjasper1 是 libjasper-dev 的依赖包。
去官网下载Source源文件https://opencv.org/releases/
解压到home/Tools下，新建build文件夹，进入build，
```sh
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
```
这里的/usr/local 是 OpenCV 的安装路径
多线程编译sudo make -j8
```sh
sudo make install
```
### 配置OpenCV环境
添加路径
```sh
sudo gedit /etc/ld.so.conf.d/opencv.conf
```
添加/usr/local/lib
保存配置
```sh
sudo ldconfig
```
配置环境
```sh
sudo gedit /etc/bash.bashrc
```
在文末添加
```sh
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH
```
保存配置
```sh
source /etc/bash.bashrc
```
更新
```sh
sudo updatedb
```
测试安装成功与否

```sh
cd home/Tools/opencv/samples/cpp/example_cmake
cmake .
make
./opencv_example
```
出现摄像头图像，即为成功。
### 安装Anno机械臂
```sh
git clone https://github.com/ps-micro/PROBOT_Anno
```
查看ros版本：
```sh
/usr/bin/rosversion -d
```
下载依赖包
```sh
sudo apt-get install ros-melodic-moveit-*
sudo apt-get install ros-melodic-industrial-*   
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers  
sudo apt-get install ros-melodic-trac-ik-kinematics-plugin   
sudo apt-get install ros-melodic-usb-cam
```
更新源列表
```sh
sudo apt-get update
```
安装可以更新的软件
```sh
sudo apt-get upgrade
```
把文件夹放入catkin_ws/src中
```sh
catkin_make
```
