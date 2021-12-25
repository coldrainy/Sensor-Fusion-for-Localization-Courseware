bash_path=/workspace/data/kitti
# bag_name=kitti_2011_10_03_drive_0027_synced.bag
bag_name=kitti_lidar_only_2011_10_03_drive_0027_synced.bag
rosbag play ${bash_path}/${bag_name} -r 0.5