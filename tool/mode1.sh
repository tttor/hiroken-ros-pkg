export ROS_MASTER_URI=http://localhost:$1
echo $ROS_MASTER_URI
#export suffix_here=date +"%Y%m%d.%H%M"
#echo $suffix_here
roslaunch hiro_common a.launch mode:=1 n_obj:=$2 n_run:=100 tidy_cfg:=/tidy_baseline.1.cfg suffix:=$3
