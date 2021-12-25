data_type=$1
evo_ape kitti src/lidar_localization/${data_type}/trajectory/ground_truth.txt src/lidar_localization/${data_type}/trajectory/laser_odom.txt -r full --plot --plot_mode xyz
