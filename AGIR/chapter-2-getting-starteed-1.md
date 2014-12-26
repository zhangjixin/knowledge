#Chapter 2 -- Getting started

##2.1~2.2 Instaling ROS

---
###**_Setup your sources.list_**

Ubuntu 14.04

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```
the *trusty* is the codename of Ubuntu14.04, if you are unsure of which Ubuntu version you're using, you can find out using this command 
```
lsb_release -a
```
###**_Setup your keys_**
```
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

```
after completing this step (apt-key should say "OK"), you can safely delete ros.key.

###**_Downloading the package lists_**
```
sudo apt-get update
```
###**_Installing the ROS packages_**
```
sudo apt-get install ros-indigo-desktop-full
```
To find available packages, use
> ```apt-cache search ros-indigo```

###**_Initialize rosdep_**

Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.

```
sudo rosdep init
```
The purpose of this command is to initialize rosdep, checking and instaling package dependencies in an OS-independent way.

Then, you must initialize the rosdep system in you account, using this command
```
rosdep update
```
Note that, the above command should be run using your normal user account, not using *sudo*.

###**_Environment setup_**

It's convenient if the ROS environment varialbles are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

source ~/.bashrc
```
You can then confirm that the environment variable are set correctly using a command like this:
```
export | grep ROS
```
###**_Getting rosinstall_**

rosinstall is a frequently used command-line tool is ROS that is distributed separately. To install this tool on Ubuntu, run:
```
sudo apt-get install python-rosinstall
```
---




