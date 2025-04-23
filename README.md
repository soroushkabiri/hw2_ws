this package contain 2 launch files.

1) turtle_tf2_demo.launch.py : this launch file build a leader and desired number of followers. user can set the num_robots_param at the begiing of launch.
for example: ros2 launch dynamic_leader_follower turtle_tf2_demo.launch.py num_robots:=5  will set a turtle1 as leader and turtle2, turtle3,turtle4,turtle5 as followers.
the followers will follow turtle1. for changing leader during process we can set current_leader parameter by changing it for /turtle_tf2_listener listener node by a code like
ros2 param set /turtle_tf2_listener current_leader turtle3  with this line we change the leader from turtle1 to turtle3. after changing leader we can see the followers
act by ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtle3/cmd_vel changing teleopkey from turtle1 to turtle3.


for bonus challenge
2) turtle_tf2_demo_random_leader.launch.py : this launch file build a leader and follower like previous launch file (desired number of robots). then changing the leader 
randomly every 20 seconds. ros2 launch dynamic_leader_follower turtle_tf2_demo_random_leader.launch.py num_robots:=6




answering hw questions:
q1) How did you implement the follower logic? 
first we write broadcaster node which get the num_robots parameter and spawn them all. then make all of them subscribe and transfer their position data to tf2. in the
listener node by using current_leader parameter make them all (excapt current leader) lookup transform to the leader frame and make a p controller by this error. 
by using callback function we check that if current_leader change the publishig robots change to delete the new leader publishing and then look up tranform by 
on_timer function to new current leader

for the random_leader part we only change listener node and adding a leadertimer that go to slect random function every 20 seconds and change the leader. 
for changing leader it set the current_leader parameter to new one so the whole process happens like you set the parameter for previously launch file

q2) How does your system detect the current leader?
by setting the parameter from terminal param set /turtle_tf2_listener current_leader turtle3


q3) What part was most challenging?
in the listener node when we set the current leader to new parameter we have to add aa rebuild publisher function inside parameter_call back to delete previously
publisher. before adding this rebuild_publisher function when we change the current_leader parameter in terminal the listener node suddenly shuuting down and
finding this problem and solving it take too much time



