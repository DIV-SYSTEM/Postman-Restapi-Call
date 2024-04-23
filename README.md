# Sending data using postman to flask server used for testing plan path
In this project, i need to call plan path module which takes robot_entry, robot_target, col_info, path_info, motion_planner_conngihlara10 as a parameter and we need to show the output as a response in postman.

#Postman data structure
postman_data  ={
    "col_info": {
        "CT_gantry": [[0, 0, 0.0], [0, 0, 0]], 
"Robot": [[0, 0, 0], [0, 0, 0]],
"Robotcart": [[-450.0, 0, -1000.0], [1.57, 0, 1.57]],
"Plane": [[420, 0, 0], [0, 0, 0]],
"PRM": [[400, 650, 50], [0, 0, 0]],
"Workspace_dimension": [210, 250, 100], 
"Workspace_position_in_RB": [[-438, -290, 0], [0, 0, 0.628319]], 
"Needles_Entry_in_robot_frame": [],
"Needles_Target_in_robot_frame": []
    },
    "path_config": {
        "ik_solution_number": 5, "Pose_type": "Ergonomic", "pull_back_length": 60, "Current_entry_target_length": 150, "Dilator_length": 150, "patient_Z_level": 100, "Entry_list_in_robot_frame": [], "Target_list_in_robot_frame": [], "Entry_with_dilators": [], "Dilators_length_list": [], "Left_home": [5, -55, 111, 0, 94, 0], "current_J": [39.47136916827013, -13.496757357781805, 130.77891965840527, 16.634453216647277, 40.40610775986151, -70.4190262729965]
    },
    "motion_planner_configlara10": {
        
"robot_type"                                 : "lara10", 
"Tool_Axis"                                  : "Z-axis",
"robot_sampling_rate"                        : 0.01    , 
"calibration_euler_order"                    : "ZYX"   ,
"calibration_value_order"                    : "XYZ"   ,
"calib_translation_pos_t6_2_t7_mm"           : [-24.46169919 ,-10.13156359, 231.92555118],
"calib_euler_angles_t6_2_t7_deg"             : [135.8122, -167.8412, 99.9816] , 
"CAMERA"                                     : "ATR"      , 
"Robot_geometry"                             : ["9998"]   ,
"Tool_geometry"                              : ["7100098"], 
"docking"                                    : "Left"     ,
"workspace_leftdock"                         : [[-700, -700, 0], [0, 0, 1700]],
"workspace_rightdock"                        : [[-700, 0, 0], [0, 700, 1700]], 
"robot_joint_limits_left_dock"               : [[-180, 180], [-120, 120], [-150, 150], [-180, 180], [-180, 180], [-180, 180]],
"robot_joint_limits_right_dock"              : [[-180, 180], [-120, 120], [-150, 150], [-180, 180], [-180, 180], [-180, 180]],
"home_pose_left_dock"                        : [-174, 70, 135, -100, -100, 75],
"home_pose_right_dock"                       : [-174, 70, 135, -100, -100, 75],
"robot_velocity"                             : 30,
"angular_velocity"                           : 0.0698132, 
"robot_workspace_limit_xyz"                  : [[-1000, 1000], [-1000, 1000], [-1000, 1000]],
"marker_ball_vectors"                        : [[-64.297, -114.921, -96.412, 1], [-6.665, -113.181, -111.442, 1],[6.946, -76.738, -76.784, 1], [-59.398, -69.309, -49.175, 1]],
"Camera_Gtcp_2_Rtcp_Transformation_mm"       :[[-0.008, 0.001, 1.000, 0.937],
                                               [-0.933, -0.360, -0.007, 0.873],
                                               [0.360, -0.933, 0.004, -0.157],
                                               [0.000, 0.000, 0.000, 1.000]],
"Robot_to_camera_Transformation_mm"          : [[0.9997183784847344, 0.011451105557060665, 0.020785473325724446, -252.65081019517172],
                                                [-0.012286221998535254, -0.4996000085627844, 0.8661690829122617, -1346.2398695579682], 
                                                [0.020303016250204087, -0.8661805260022801, -0.4993186196263435, 835.6684717747726],
                                                [0.0, 0.0, 0.0, 1.0]]


    },
     "robot_entry_": {
        "robot_entry_": [396.46516975603464, 307.0, 148.15325108927067]
    },
    "MOVE_PHYSICAL_ROBO": {
        "MOVE_PHYSICAL_ROBO" : "False"
    },
    "robot_target_": {
        "robot_target_" : [373, 307, 0]
    },
    "current_joint_angles" : {
        "current_joint_angles" : [0.4389431407455767, 4.346514294123517, 125.4750837670467, 50.04662521815736, 68.70165797549248, -135.00390460147227]
    }
}
#TypeError: Object of type ndarray is not JSON serializable:
I have returned the values in json and one variable  contains nested lists with NumPy arrays which is not json serializable for that i need to convert it into json serializable.

#Function to make variable "var"  json serializable:
def convert_numpy_to_list(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, list):
        return [convert_numpy_to_list(item) for item in obj]
    else:
        return obj
var = convert_numpy_to_list(var)
     
