# LIO-SAM Mapper

This is an offline version of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) that produces a determenstic output, through exchanging messages/data using pointers instead of topics. It is configured to run with ouster lidar packets and pointclouds. This mapper can output different pointclouds to PCD files as well as a CSV file containing the poses at each keyframe.

## Installation

Todo.


## Demo


- Download and unzip [court_yard_stroll_filtered.zip](https://drive.google.com/file/d/1kS37vxyYExPopwk1U28UVPDy4ultXzKo/view?usp=sharing).
- Modify some parameters in [param.yaml](https://github.com/FAIRSpace-AdMaLL/liosam_mapper/blob/offline/config/params.yaml).
    * Set `readBag` to the path of court_yard_stroll_filtered.zip.
    * Set `SaveDire` to a directory where the output files could be saved.
    * Set `pointCloudTopic` to `/os_cloud_node/points`.
- Run 
```
roslaunch lio_sam run.launch
```


## Parameters

Below are the additional parameters added, information about the other parameters might be found in [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM). 

| Parameter | Type | Description |
| --- | --- | --- |
| readBag | String | Rosbag path
| saveDir | String | Saving directory
| saveToRosbag | Bool | Set to True to save global map and trajectory to a rosbag
| saveTrajectoryCSV | Bool | Set to True to save trajectory to CSV
| saveRawPCD | Bool | Set to True to save raw lidar pointclouds to PCDc (lidar frame)
| saveDeskewedPCD | Bool | Set to True to save deskewed pointclouds to PCDs (lidar frame)
| saveRegisteredCloudPCD | Bool | Set to True to save registered pointclouds to PCDs (map frame)
| saveRegisteredFeaturesPCD | Bool | Set to True to save registered feature pointclouds to PCDs (map frame)
| LMOMaxIterations | Int | Max iterations count for lidar odometry optimisation
