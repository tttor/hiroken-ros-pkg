export ROS_MASTER_URI=http://localhost:$1
echo $ROS_MASTER_URI
roscore -p $1
