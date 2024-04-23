from flask import Flask, jsonify, request, render_template_string
import json

from planpath_funapi import *

app = Flask(__name__)



@app.route('/callFunction', methods=['POST'])
def call_function():
    
    data = request.json

    
    col_info = data['col_info']
    path_config = data['path_config']
    motion_planner_config = data['motion_planner_configlara10']
    robot_entry = data['robot_entry_']
    robot_entry_ = robot_entry.get('robot_entry_')
    
    
    robot_target = data['robot_target_']
    robot_target_ = robot_target.get('robot_target_')
    
    current_joint_angle = data['current_joint_angles']
    current_joint_angles = current_joint_angle.get('current_joint_angles')
    MOVE_PHYSICAL_ROBOT = data['MOVE_PHYSICAL_ROBO']
    MOVE_PHYSICAL_ROBO = MOVE_PHYSICAL_ROBOT.get('MOVE_PHYSICAL_ROBO')
    if MOVE_PHYSICAL_ROBO == "False":
        MOVE_PHYSICAL_ROBO = False
    else :
        MOVE_PHYSICAL_ROBO = True
    print(type(robot_entry_))
    print(type(robot_target_))
    print(type(current_joint_angles))
    print(type(MOVE_PHYSICAL_ROBO))
    print(type(col_info))
    print(type(path_config))
    print(type(motion_planner_config))
    #function_name = data['Function_name']
    Robot_to_camera_Transformation_mm = [[0.8327183784847344, 0.777451105557060665, 0.121785473325724446, -666.65081019517172],
                                                [-0.064286221998535254, -0.4989000085627844, 0.5361690829122617, -1886.2398695579682], 
                                                [0.222303016250204087, -0.6631805260022801, -0.7878318619626343, 368.6684717747726],
                                                [0.0, 0.0, 0.0, 1.0]]
    res = plan_path_funapi(robot_entry_,robot_target_,col_info,current_joint_angles,
              path_config,motion_planner_config,MOVE_PHYSICAL_ROBO)
    
    result = jsonify(res)
    return result
    


if __name__ == '__main__':
    app.run(debug=True)