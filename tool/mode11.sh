export ROS_MASTER_URI=http://localhost:$1
echo $ROS_MASTER_URI
roslaunch hiro_common a.launch  mode:=11 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline n_obj:=$2 n_run:=$3 epsth:=$4

