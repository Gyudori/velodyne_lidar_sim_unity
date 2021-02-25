# Velodyne Lidar Simulator(Unity)
Velodyen lidar simulator using multi-threading raycast in Unity.\
Implemented sensor model:
- Geometrical sensor model
  - basic model(baseline)
  - precise model
- Radiometrical sensor model
  - basic model(baseline)
  - precise model
  - 
After the generation of simulation data, we evaluate the results by comparing with real lidar data.

Unity lidar simulation scene\
<img src="./images/Unity_play_scene.png" width="700"/>


The result of geometrical simulation.
- White: real
- Red: basic model
- Green: precise model
<img src="./images/geometric_overlay.jpg" width="500"/>

The result of geometric comparison with real lidar data.\
<img src="./images/distance_difference.jpg" width="500"/>

The result of raidometric simulation.\
<img src="./images/point_cloud_intensity.jpg" width="500"/>

The result of radiometric comparison with real lidar data.\
<img src="./images/intensity_difference.jpg" width="500"/>
