# dwa_bcr_bot
bcr_bot(from black coffee robotics) simulated in gazebo ros2, using kinect depth camera and getting depth images from it in pointcloud2 images message format, downsampling the data to see obstacles in realtime.

#starting the simulation
launch the gazebo.launch from the workspace in terminal using the command
"ros2 launch bcr_bot gazebo.launch"

#open rviz2 using the command
"rviz2"

#start the python file "point_cloud_processor.py" in vs code or another terminal
"python3 point_cloud_processor.py"

# optimising the simulation
~check the frame name to kinect_camera
~if you can't see any pointclouds, check that pointcloud2 data is added on the left menu of rviz, if not then on the lower left ADD and click pointcloud2
~add the topic name /grouped_points
~for more points, in the python file increase the value of call_size
