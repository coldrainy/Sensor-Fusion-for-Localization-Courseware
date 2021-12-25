data_type=$1
evo_rpe kitti src/lidar_localization/${data_type}/trajectory/ground_truth.txt src/lidar_localization/${data_type}/trajectory/laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz
