# pcd_loader_ros
***pcd_loader_ros*** is a ROS package for loading pcd files on rviz for visualization. by loading the map to rviz it can be used for multipurposes such as Localization and path planning.
the package was inpired by **hdl_localization project by koide3**

## Requirements
- PCL 1.7
- pcl_ros (ros package)


## Parameters
Parameters like directory and downsampling resolutions are located in /launch/map_load.launch file<br>

## Instructions
Only pcd files can be treated. so if you want to export other extensions -ex) .xyz, .las<br>
You might want to convert it into pcd files using CloudCompare or https://github.com/clausmichele/xyz2pcd

```bash
roslaunch pcd_loader_ros map_load.launch
```



