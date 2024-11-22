from mjcf_urdf_simple_converter import convert

# or, if you are using it in your ROS package and would like for the mesh directories to be resolved correctly, set meshfile_prefix, for example:
convert("/home/esteb37/ros2_ws/src/dextrous_hand/data/assets/hh_hand.xml", "/home/esteb37/ros2_ws/src/dextrous_hand/data/assets/urdf/hh_hand.urdf", asset_file_prefix="$(find dextrous_hand)/data/assets/urdf/")