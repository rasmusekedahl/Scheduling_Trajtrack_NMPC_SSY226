import yaml
import json
import os, glob


def set_up_Json_files(file_to_parse,vehicles):
    with open(file_to_parse,'r') as read_file:
        data = json.load(read_file)
        current_dir = os.getcwd()
        gpss_directory = '/'.join(current_dir.split('/')[:-1])
        # here I am creating a json file for each node in the graph that represents my plant and saving it
        # (hopefully in the right format) in the frames folder
        nodes = data['test_data']['nodes']
        node_files = glob.glob('{}/gpss_scenario/frames/*'.format(gpss_directory))
        for f in node_files:
            os.remove(f)
        for node_name in nodes:
            node_buffer = {
                "child_frame_id": node_name,
                "parent_frame_id": "world",
                "transform": {
                    "translation": {
                        "x": nodes[node_name]["x"],
                        "y": nodes[node_name]["y"],
                        "z": 0.0
                    },
                    "rotation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0
                    }
                },
                "extra_data": {
                    "active": False,
                    "zone": 0.5,
                    "next": nodes[node_name]["next"],
                    "frame_type": "waypoint"
                }
            }
            json_node = json.dumps(node_buffer, indent= 4)
            with open("{}/gpss_scenario/frames/{}.json".format(gpss_directory,node_name),"w") as outfile:
                outfile.write(json_node)
        # here i am adding the lines to launch the robots in the "launch_atrs.sh" file
        # for now i only want the active vehicles to take part of the simulation
        with open("{}/launch_atrs.sh".format(gpss_directory),'w') as write_file:
            for i in vehicles[:-1]:
                write_file.write("ros2 run gpss_controller gpss_controller --ros-args -p atr_id:={} -p simulation:=True &\\ \n".format(i))
        # i need to get the last vehicle aside from the others cause there is no \ in the string
            write_file.write(
                "ros2 run gpss_controller gpss_controller --ros-args -p atr_id:={} -p simulation:=True &".format(
                    vehicles[-1]))
        # now I am going to modify the teleport_atrs.sh so that the vehicles are moved to their initial location
        with open("{}/teleport_atrs.sh".format(gpss_directory),'w') as write_file:
            for i in vehicles:
                write_file.write('ros2 topic pub --once /%s/teleport geometry_msgs/msg/Point \"{\'x\':%s, \'y\':%s}\" \n' % (i,nodes[i]["x"],nodes[i]["y"]))
        # now I am creating a json file where i stored all the atrs names that will be used to launch the vehicles in ROS
        atr_names = [int(i) for i in vehicles]
        with open("{}/gpss_scenario/config/atrs_list.json".format(gpss_directory),"w") as write_file:
                json.dump(atr_names,write_file)

def set_up_Rviz_Config(input_folder, vehicles):
    current_dir = os.getcwd()
    gpss_directory = '/'.join(current_dir.split('/')[:-1])
    node_files = glob.glob(input_folder)
    nodes = [i.split('/')[-1].split('.')[0] for i in node_files]
    # now comes the TOUGH part....the Rviz config file
    rviz_config = {}
    rviz_config.update({"Panels": [   {   "Class": "rviz_common/Displays",
                  "Help Height": 78,
                  "Name": "Displays",
                  "Property Tree Widget": {   "Expanded": [   "/Grid1/Offset1",
                                                              "/TF1",
                                                              "/TF1/Frames1",
                                                              "/ATRModel121",
                                                              "/MarkerArray1/Topic1"],
                                              "Splitter Ratio": 0.6529411673545837},
                  "Tree Height": 719},
              {"Class": "rviz_common/Selection", "Name": "Selection"},
              {   "Class": "rviz_common/Tool Properties",
                  "Expanded": ["/2D Goal Pose1", "/Publish Point1"],
                  "Name": "Tool Properties",
                  "Splitter Ratio": 0.5886790156364441},
              {   "Class": "rviz_common/Views",
                  "Expanded": ["/Current View1"],
                  "Name": "Views",
                  "Splitter Ratio": 0.5},
              {   "Class": "rviz_common/Time",
                  "Experimental": False,
                  "Name": "Time",
                  "SyncMode": 0,
                  "SyncSource": ""}]})
    tree = {
        'world_origin':{
            'world':{
                    "atr_{}_base_link_axis_x".format(vehicle):{
                        "atr_{}_base_link_axis_y".format(vehicle):{
                            "atr_{}_base_link_axis_z".format(vehicle):{
                                "atr_{}_base_link".format(vehicle):{
                                    "atr_{}_back_left_wheel".format(vehicle):{},
                                    "atr_{}_back_right_wheel".format(vehicle): {},
                                    "atr_{}_col_front_chassis".format(vehicle): {},
                                    "atr_{}_col_front_left".format(vehicle): {},
                                    "atr_{}_col_front_right".format(vehicle): {},
                                    "atr_{}_col_rear_left".format(vehicle): {},
                                    "atr_{}_col_rear_right".format(vehicle): {},
                                    "atr_{}_loop_front_center".format(vehicle): {},
                                    "atr_{}_loop_front_right".format(vehicle): {},
                                    "atr_{}_loop_rear_left".format(vehicle): {},
                                    "atr_{}_loop_rear_right".format(vehicle): {},
                                    "atr_{}_pillar_front_right".format(vehicle): {
                                        "atr_{}_table".format(vehicle):{
                                            "atr_{}_pillar_back_left".format(vehicle):{},
                                            "atr_{}_pillar_back_right".format(vehicle): {},
                                            "atr_{}_pillar_front_left".format(vehicle): {}
                                        },
                                    },
                                    "atr_{}_sw_arm_left_1".format(vehicle): {
                                        "atr_{}_sw_arm_left_2".format(vehicle): {
                                            "atr_{}_sw_arm_left_3".format(vehicle): {
                                                "atr_{}_sw_arm_left_4".format(vehicle): {
                                                    "atr_{}_front_left_swivel_wheel".format(vehicle): {}
                                                }
                                            }
                                        }
                                    },
                                    "atr_{}_sw_arm_right_1".format(vehicle): {
                                        "atr_{}_sw_arm_right_2".format(vehicle): {
                                            "atr_{}_sw_arm_right_3".format(vehicle): {
                                                "atr_{}_sw_arm_right_4".format(vehicle): {
                                                    "atr_{}_front_right_swivel_wheel".format(vehicle): {}
                                                }
                                            }
                                        }
                                    }

                                }
                            }
                        }
                    }
                for vehicle in vehicles
            }
        }
    }

    frames = {
        "All Enabled": False,
        "teaching_marker": {"Value": False},
        "world": {"Value": False},
        "world_origin": {"Value": False},
    }
    for vehicle in vehicles:
        frames.update({
                          "atr_{}_back_left_wheel".format(vehicle): {"Value": False},
                          "atr_{}_back_right_wheel".format(vehicle): {"Value": False},
                          "atr_{}_base_link".format(vehicle): {"Value": False},
                          "atr_{}_base_link_axis_x".format(vehicle): {"Value": False},
                          "atr_{}_base_link_axis_y".format(vehicle): {"Value": False},
                          "atr_{}_base_link_axis_z".format(vehicle): {"Value": False},
                          "atr_{}_col_front_chassis".format(vehicle): {"Value": False},
                          "atr_{}_col_front_left".format(vehicle): {"Value": False},
                          "atr_{}_col_front_right".format(vehicle): {"Value": False},
                          "atr_{}_col_rear_left".format(vehicle): {"Value": False},
                          "atr_{}_col_rear_right".format(vehicle): {"Value": False},
                          "atr_{}_front_left_swivel_wheel".format(vehicle): {"Value": False},
                          "atr_{}_front_right_swivel_wheel".format(vehicle): {"Value": False},
                          "atr_{}_loop_front_center".format(vehicle): {"Value": False},
                          "atr_{}_loop_front_right".format(vehicle): {"Value": False},
                          "atr_{}_loop_rear_left".format(vehicle): {"Value": False},
                          "atr_{}_loop_rear_right".format(vehicle): {"Value": False},
                          "atr_{}_pillar_back_left".format(vehicle): {"Value": False},
                          "atr_{}_pillar_back_right".format(vehicle): {"Value": False},
                          "atr_{}_pillar_front_left".format(vehicle): {"Value": False},
                          "atr_{}_pillar_front_right".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_left_1".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_left_2".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_left_3".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_left_4".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_right_1".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_right_2".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_right_3".format(vehicle): {"Value": False},
                          "atr_{}_sw_arm_right_4".format(vehicle): {"Value": False},
                          "atr_{}_table".format(vehicle): {"Value": False}

        })

    robot_models = {
        'atr_{}'.format(vehicle):{   "Alpha": 1,
                                                 "Class": "rviz_default_plugins/RobotModel",
                                                 "Collision Enabled": False,
                                                 "Description File": "",
                                                 "Description Source": "Topic",
                                                 "Description Topic": {   "Depth": 5,
                                                                          "Durability Policy": "Volatile",
                                                                          "History Policy": "Keep Last",
                                                                          "Reliability Policy": "Reliable",
                                                                          "Value": "/robot_description_atr_{}".format(vehicle)},
                                                 "Enabled": True,
                                                 "Links": {   "All Links Enabled": True,
                                                              "Expand Joint Details": False,
                                                              "Expand Link Details": False,
                                                              "Expand Tree": False,
                                                              "Link Tree Style": "Links in Alphabetic Order",
                                                              "atr_{}_back_left_wheel".format(vehicle): {   "Alpha": 1,
                                                                                            "Show Axes": False,
                                                                                            "Show Trail": False,
                                                                                            "Value": True},
                                                              "atr_{}_back_right_wheel".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False,
                                                                                             "Value": True},
                                                              "atr_{}_base_link".format(vehicle): {   "Alpha": 1,
                                                                                      "Show Axes": False,
                                                                                      "Show Trail": False,
                                                                                      "Value": True},
                                                              "atr_{}_base_link_axis_x".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False},
                                                              "atr_{}_base_link_axis_y".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False},
                                                              "atr_{}_base_link_axis_z".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False},
                                                              "atr_{}_col_front_chassis".format(vehicle): {   "Alpha": 1,
                                                                                              "Show Axes": False,
                                                                                              "Show Trail": False},
                                                              "atr_{}_col_front_left".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False},
                                                              "atr_{}_col_front_right".format(vehicle): {   "Alpha": 1,
                                                                                            "Show Axes": False,
                                                                                            "Show Trail": False},
                                                              "atr_{}_col_rear_left".format(vehicle): {   "Alpha": 1,
                                                                                          "Show Axes": False,
                                                                                          "Show Trail": False},
                                                              "atr_{}_col_rear_right".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False},
                                                              "atr_{}_front_left_swivel_wheel".format(vehicle): {   "Alpha": 1,
                                                                                                    "Show Axes": False,
                                                                                                    "Show Trail": False,
                                                                                                    "Value": True},
                                                              "atr_{}_front_right_swivel_wheel".format(vehicle): {   "Alpha": 1,
                                                                                                     "Show Axes": False,
                                                                                                     "Show Trail": False,
                                                                                                     "Value": True},
                                                              "atr_{}_loop_front_center".format(vehicle): {   "Alpha": 1,
                                                                                              "Show Axes": False,
                                                                                              "Show Trail": False},
                                                              "atr_{}_loop_front_right".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False},
                                                              "atr_{}_loop_rear_left".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False},
                                                              "atr_{}_loop_rear_right".format(vehicle): {   "Alpha": 1,
                                                                                            "Show Axes": False,
                                                                                            "Show Trail": False},
                                                              "atr_{}_pillar_back_left".format(vehicle): {   "Alpha": 1,
                                                                                             "Show Axes": False,
                                                                                             "Show Trail": False,
                                                                                             "Value": True},
                                                              "atr_{}_pillar_back_right".format(vehicle): {   "Alpha": 1,
                                                                                              "Show Axes": False,
                                                                                              "Show Trail": False,
                                                                                              "Value": True},
                                                              "atr_{}_pillar_front_left".format(vehicle): {   "Alpha": 1,
                                                                                              "Show Axes": False,
                                                                                              "Show Trail": False,
                                                                                              "Value": True},
                                                              "atr_{}_pillar_front_right".format(vehicle): {   "Alpha": 1,
                                                                                               "Show Axes": False,
                                                                                               "Show Trail": False,
                                                                                               "Value": True},
                                                              "atr_{}_sw_arm_left_1".format(vehicle): {   "Alpha": 1,
                                                                                          "Show Axes": False,
                                                                                          "Show Trail": False,
                                                                                          "Value": True},
                                                              "atr_{}_sw_arm_left_2".format(vehicle): {   "Alpha": 1,
                                                                                          "Show Axes": False,
                                                                                          "Show Trail": False,
                                                                                          "Value": True},
                                                              "atr_{}_sw_arm_left_3".format(vehicle): {   "Alpha": 1,
                                                                                          "Show Axes": False,
                                                                                          "Show Trail": False,
                                                                                          "Value": True},
                                                              "atr_{}_sw_arm_left_4".format(vehicle): {   "Alpha": 1,
                                                                                          "Show Axes": False,
                                                                                          "Show Trail": False,
                                                                                          "Value": True},
                                                              "atr_{}_sw_arm_right_1".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False,
                                                                                           "Value": True},
                                                              "atr_{}_sw_arm_right_2".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False,
                                                                                           "Value": True},
                                                              "atr_{}_sw_arm_right_3".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False,
                                                                                           "Value": True},
                                                              "atr_{}_sw_arm_right_4".format(vehicle): {   "Alpha": 1,
                                                                                           "Show Axes": False,
                                                                                           "Show Trail": False,
                                                                                           "Value": True},
                                                              "atr_{}_table".format(vehicle): {   "Alpha": 1,
                                                                                  "Show Axes": False,
                                                                                  "Show Trail": False,
                                                                                  "Value": True},
                                                              "world": {   "Alpha": 1,
                                                                           "Show Axes": False,
                                                                           "Show Trail": False}},
                                                 "Mass Properties": {   "Inertia": False,
                                                                        "Mass": False},
                                                 "Name": "ATRModel{}".format(vehicle),
                                                 "TF Prefix": "",
                                                 "Update Interval": 0,
                                                 "Value": True,
                                                 "Visual Enabled": True}
        for vehicle in vehicles
    }

    displays = [
                     {"Alpha": 0.5,
                      "Cell Size": 1,
                      "Class": "rviz_default_plugins/Grid",
                      "Color": "0; 0; 0",
                      "Enabled": True,
                      "Line Style": {"Line Width": 0.029999999329447746,
                                     "Value": "Lines"},
                      "Name": "Grid",
                      "Normal Cell Count": 0,
                      "Offset": {"X": 27,
                                 "Y": 5,
                                 "Z": 0},
                      "Plane": "XY",
                      "Plane Cell Count": 60,
                      "Reference Frame": "<Fixed Frame>",
                      "Value": True},
                     {"Class": "rviz_default_plugins/TF",
                      "Enabled": True,
                      "Frame Timeout": 5,
                      "Frames":frames,
                      "Marker Scale": 3,
                      "Name": "TF",
                      "Show Arrows": False,
                      "Show Axes": True,
                      "Show Names": True,
                      "Tree":tree,
                      "Update Interval": 0,
                      "Value": True},
                    {"Class": "rviz_default_plugins/InteractiveMarkers",
                     "Enable Transparency": True,
                     "Enabled": True,
                     "Interactive Markers Namespace": "/scene_manipulation_marker_server",
                     "Name": "TeachingMarker",
                     "Show Axes": False,
                     "Show Descriptions": False,
                     "Show Visual Aids": False,
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "ZoneMarkers",
                     "Namespaces": {" ": True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/zone_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "EnabledPathMarkers",
                     "Namespaces": {" ":True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/path_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "ConcaveObstacles",
                     "Namespaces": {" ":True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/gpss_concave_obstacle_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "ConvexObstacles",
                     "Namespaces": {" ":True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/gpss_convex_obstacle_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "ARTagMarkers",
                     "Namespaces":{" ": True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/ar_tag_markers"},
                     "Value": True},
                    # {"Class": "rviz_default_plugins/MarkerArray",
                    #  "Enabled": True,
                    #  "Name": "FactoryBeamsMarkers",
                    #  "Namespaces": {" ": True},
                    #  "Topic": {"Depth": 5,
                    #            "Durability Policy": "Volatile",
                    #            "History Policy": "Keep Last",
                    #            "Reliability Policy": "Reliable",
                    #            "Value": "/gpss_factory_beams_markers"},
                    #  "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "ControllersState",
                     "Namespaces": {" ": True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/controller_state_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "PlannedPaths",
                     "Namespaces": {},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/planned_path_markers"},
                     "Value": True},

                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "MarkerArray",
                     "Namespaces": {},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/gpss_convex_agv_markers"},
                     "Value": True},
                    {"Class": "rviz_default_plugins/MarkerArray",
                     "Enabled": True,
                     "Name": "MarkerArray",
                     "Namespaces": {" ": True},
                     "Topic": {"Depth": 5,
                               "Durability Policy": "Volatile",
                               "History Policy": "Keep Last",
                               "Reliability Policy": "Reliable",
                               "Value": "/gpss_concave_agv_markers"},
                     "Value": True},
            ]
    for robot_model in robot_models:
        displays.append(robot_models[robot_model])

    # DO I REALLY NEED TO PUT THE NODES IN THE CONFIG? OR DOES IT UPLOAD THEM THROUGH THE JSON?
    for node in nodes:
        frames.update({node:{"Value":True}})
    rviz_config.update({
        "Visualization Manager":{
            "Class": "",
            "Displays": displays,
            "Enabled": True,
            "Global Options": {"Background Color": "119; 118; 123",
                               "Fixed Frame": "world",
                               "Frame Rate": 30},
            "Name": "root",
            "Tools": [{"Class": "rviz_default_plugins/Interact",
                       "Hide Inactive Objects": True},
                      {"Class": "rviz_default_plugins/MoveCamera"},
                      {"Class": "rviz_default_plugins/Select"},
                      {"Class": "rviz_default_plugins/FocusCamera"},
                      {"Class": "rviz_default_plugins/Measure",
                       "Line color": "128; 128; 0"},
                      {"Class": "rviz_default_plugins/SetInitialPose",
                       "Covariance x": 0.25,
                       "Covariance y": 0.25,
                       "Covariance yaw": 0.06853891909122467,
                       "Topic": {"Depth": 5,
                                 "Durability Policy": "Volatile",
                                 "History Policy": "Keep Last",
                                 "Reliability Policy": "Reliable",
                                 "Value": "/initialpose"}},
                      {"Class": "rviz_default_plugins/SetGoal",
                       "Topic": {"Depth": 5,
                                 "Durability Policy": "Volatile",
                                 "History Policy": "Keep Last",
                                 "Reliability Policy": "Reliable",
                                 "Value": "/goal_pose"}},
                      {"Class": "rviz_default_plugins/PublishPoint",
                       "Single click": True,
                       "Topic": {"Depth": 5,
                                 "Durability Policy": "Volatile",
                                 "History Policy": "Keep Last",
                                 "Reliability Policy": "Reliable",
                                 "Value": "/clicked_point"}}],
            "Transformation": {"Current": {"Class": "rviz_default_plugins/TF"}},
            "Value": True,
            "Views": {"Current": {"Class": "rviz_default_plugins/Orbit",
                                  "Distance": 16.70012855529785,
                                  "Enable Stereo Rendering": {"Stereo Eye Separation": 0.05999999865889549,
                                                              "Stereo Focal Distance": 1,
                                                              "Swap Stereo Eyes": False,
                                                              "Value": False},
                                  "Focal Point": {"X": 14.88005542755127,
                                                  "Y": 11.00747013092041,
                                                  "Z": -4.938854217529297},
                                  "Focal Shape Fixed Size": True,
                                  "Focal Shape Size": 0.05000000074505806,
                                  "Invert Z Axis": False,
                                  "Name": "Current View",
                                  "Near Clip Distance": 0.009999999776482582,
                                  "Pitch": 0.5197970867156982,
                                  "Target Frame": "<Fixed Frame>",
                                  "Value": "Orbit (rviz_default_plugins)",
                                  "Yaw": 3.9823548793792725},
                      "Saved": None}
        },


        "Window Geometry": {"Displays": {"collapsed": False},
                            "Height": 1016,
                            "Hide Left Dock": False,
                            "Hide Right Dock": False,
                            "QMainWindow State": "000000ff00000000fd00000004000000000000027c0000035afc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000035a000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000035afc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d0000035a000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007360000003efc0100000002fb0000000800540069006d0065010000000000000736000002fb00fffffffb0000000800540069006d006501000000000000045000000000000000000000039f0000035a00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000",
                            "Selection": {"collapsed": False},
                            "Time": {"collapsed": False},
                            "Tool Properties": {"collapsed": False},
                            "Views": {"collapsed": False},
                            "Width": 1846,
                            "X": 74,
                            "Y": 27}
    })

    with open("{}/gpss_scenario/config/gpss.rviz".format(gpss_directory),"w") as write_file:
        yaml.dump(rviz_config,write_file)

if __name__ == "__main__":
    # file_to_parse = "/home/sabino/Documents/SP_ComSat/test_cases/Volvo_gpss_2.json"
    # file_to_parse = "/home/sabino/Documents/SP_ComSat/test_cases/MM_1538_0.2_20_21.json"
    file_to_parse = "/home/sabino/Documents/gpss/SP_ComSat/test_cases/MM_301010_0.5_7_7.json"
