# marker-based-leader-follower-controller
PID controller for leader follower mobile robots architecture using artag marker with ROS
PID controller for leader follower mobile robots formation using artag marker with ROS.

Summit XL mobile robot was used as the leader who carry the marker and RB1-base mobile robot was used as the follower.

The whole process was conducted with ubuntu 16.04 and ROS.

You need to install ar_track_alvar package in your workspace and to add the file rb1_artags_detect.launch to this directory:src/ar_track_alvar/ar_track_alvar/launch

    1) Copy the project to your workspace

    2) catkin_make

    3) roslaunch ar_tag_follower.launch

PID_param.yaml is used to set up Kp, Ki and Kd for PID controller

tag_follower.launch is used to set up the maximum speed of the follower and the threshold distance between leader and follower

Feel free to adapt this project for your necessity.

Video link: https://www.youtube.com/watch?v=EcpVjaegKdg

@article{

title={PID tuning for vision based mobile robots formation control}

author={ Ali Sayah, Jean-François Brethé}

conference {ICROM}

Year {2021}

}
