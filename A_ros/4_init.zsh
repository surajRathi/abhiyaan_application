rosrun turtlesim turtlesim_node &

sleep 1

rosservice call /spawn "x: 10.0
y: 10.0
theta: 0.0
name: 'abhiyaan'"

sleep 1
