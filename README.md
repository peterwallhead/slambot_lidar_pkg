_A ROS2 package specifically for the RPLiDAR C1 - DTOF LiDAR 360&deg; Laser Range Scanner (purchased from [Core Electronics](https://core-electronics.com.au/rplidar-c1-dtof-lidar-360-laser-range-scanner-12m-ip54.html))._

Acknowledging that Slamtech offers the official [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) package for this device.

### Usage (ROS2 "Humble")
Clone this repository into your workspace's src directory
```
git clone https://github.com/peterwallhead/slambot_lidar_pkg.git
```

Build the package
```
colcon build --packages-select slambot_lidar_pkg
```

Source and run from your workspace, ensuring you've selected the correct USB port
```
source ~/.bashrc
ros2 run slambot_lidar_pkg lidar_publisher -- --lidar_port '~/dev/ttyUSB0'
```
