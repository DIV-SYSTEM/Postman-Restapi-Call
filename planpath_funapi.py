


"""Imports """
import time
import math
import numpy as np
import json
from scipy.spatial.transform import Rotation as scipyRotation
from skspatial.objects import Plane
import transforms3d 
import pybullet 
from pybullet_utils import bullet_client
import pandas
# import platform
import threading
import urllib3
import plotly.graph_objs as go
import open3d as o3d
import transforms3d as t3d
import sys
import traceback
# path_flag = 0 -> successful plan (pass)
# path_flag = 1 -> final IK does not exist (fail)
# path_flag = 2 -> approaching singularity (fail)
# path_flag = 3 -> joint limit (fail)
# path_flag = 4 -> collision with plane/PRM (fail)
# path_flag = 5 -> self collision (fail)
# path_flag = 6 -> approaching singularity but possible after going home (pass)
# path_flag = 7 -> collision with needle implant can be overridden (pass)


def check_feasibility_l1(joint_vals, path_info,collision_info,path_config,
                            mp_collision_detection_without_gui):
    """
    Checks if the returned joint_vals are feasible
    """
    path_flag = 0
    if joint_vals == 0 :
        print("Final Pose Ik doesn't Exist")
        path_flag = 1
        return path_flag,[]  
    if joint_vals == 1 :
        print("Robot approaching singularity")
        path_flag = 2
        return path_flag,[]
    if joint_vals == 2 :
        print("Robot approaching joint limits")
        path_flag = 3
        return path_flag,[]
    if joint_vals == 3 :
        print("Robot approaching high joint velocity")
        path_flag = 4
        return path_flag,[]
    st2 = time.time()


    col_res = mp_collision_detection_without_gui.pre_check([0,[],joint_vals],path_config)
    print(" L1 Collision Time...: ", time.time() - st2)


    if col_res[1]:
        print("Collision with plane")
        path_flag = 5
        path_info =[]
        return path_flag,path_info

    if col_res[2]:
        print("Self-collision")
        path_flag = 6
        path_info=[]
        return path_flag,path_info
    
    if col_res[3]:
        print("PRM Collision")
        path_flag = 8
        path_info=[]
        return path_flag,path_info
    
    if col_res[0][0]:
        print("WARNING: Planned path might cause collision with the needles; gap: ", col_res[0])
        path_flag = 7
        return path_flag,path_info


    return path_flag,path_info

def check_feasibility_mis(joint_angles_stack,pull_back_collision_check,
                                collision_info,path_config,
                                mp_collision_detection_without_gui):
    """
    Checks if the returned joint_vals are feasible
    """
    path_flag = 0
    if isinstance(joint_angles_stack,int):
        if joint_angles_stack == 0:
            print("Final Pose Ik doesn't Exist")
            path_flag = 1
            return path_flag,[]
        if joint_angles_stack == 1:
            print("Robot approaching singularity")
            path_flag = 2
            return path_flag,[]
        if joint_angles_stack == 2:
            print("Robot approaching joint limits")
            path_flag = 3
            return path_flag,[]     
        if joint_angles_stack == 3:
            print("Robot approaching high joint velocity")
            path_flag = 4
            return path_flag,[]      
        if joint_angles_stack == -1:
            print(" PATH generation failed for MIS")
            path_flag = 9
            return path_flag,[]
    if path_flag==0:
        st1 = time.time()

        col_state = mp_collision_detection_without_gui.pre_check(pull_back_collision_check,path_config)
        print("Collision Time...: ", time.time() - st1)
        print(f'col_state{col_state}')
        # input("df")

        if col_state[1]==True:
            print(" plane Collison exist")
            path_flag = 5
            return path_flag,[]
        if col_state[2]==True:
            print(" self Collison exist")
            path_flag = 6
            return path_flag,[]
        if col_state[3]==True:
            print(" PRM Collison exist")
            path_flag = 8
            return path_flag,[]
        
        if col_state[0][0]:
            print("WARNING: Planned path might cause collision with the needles; gap: ", col_state[0])
            path_flag = 7
            return path_flag,joint_angles_stack

        if col_state[0][0]==False and col_state[1]==False and col_state[2]==False and col_state[3]==False:
            return path_flag,joint_angles_stack
        
def plan_path_mis(Motion_planner,robot_entry_,robot_target_,Entry_list,Target_list,current_joint_angles,
                            collision_info,pose_collision_detection,path_config,
                            mp_collision_detection_without_gui,libraries):

    """
    Motion Planner(mis) Starts Here
    """

    joint_list_ang_vel_time,needle_entry_list,needle_target_list,pull_back_collision_check,path_info_mis,final_pose_n_coll_flag = Motion_planner.PlanMIS(robot_entry_,robot_target_,Entry_list,Target_list,current_joint_angles,collision_info,
                                                                                                                                    pose_collision_detection,path_config,libraries)

    if isinstance(joint_list_ang_vel_time, list):
        joint_angles_stack = joint_list_ang_vel_time[0]

    else:
        joint_angles_stack = joint_list_ang_vel_time

    path_flag,joint_angles_stack= check_feasibility_mis(joint_angles_stack,pull_back_collision_check,
                                                        collision_info,path_config,
                                                        mp_collision_detection_without_gui)
    
    if not isinstance(joint_list_ang_vel_time, list):
        joint_list_ang_vel_time = [[],[],[]] 

    return path_flag,joint_list_ang_vel_time,needle_entry_list,needle_target_list,path_info_mis,final_pose_n_coll_flag

def plan_path_l1(Motion_planner,robot_entry_,robot_target_,Entry_list,Target_list,current_joint_angles,
                            collision_info,pose_collision_detection,path_config,
                            mp_collision_detection_without_gui,libraries):
    """
    Motion Planner(L1) Starts Here
    """
    patient_Z_level      = path_config["patient_Z_level"]

    joint_list_ang_vel_time,path_info,final_pose_n_coll_flag = Motion_planner.PlanL1MIS(robot_entry_,robot_target_,Entry_list,
                                            Target_list,current_joint_angles,collision_info,
                                            pose_collision_detection,path_config,patient_Z_level,
                                            libraries,movetype=3)
    
    path_flag = 0
    path_flag, path_info = check_feasibility_l1(joint_list_ang_vel_time[0], path_info,
                                                collision_info,path_config,
                                                mp_collision_detection_without_gui)   
    
    return path_flag, joint_list_ang_vel_time, [], [], path_info, final_pose_n_coll_flag

def plan_path(Motion_planner,robot_entry_,robot_target_,collision_info,current_joint_angles,
              path_config,pose_collision_detection,
              mp_collision_detection_without_gui,libraries):
    """
    Function algorithm that decides whether to use MIS or L1 planner
    """                                 
    Entry_list  = path_config["Entry_list_in_robot_frame"]
    Target_list = path_config["Target_list_in_robot_frame"]
    # Docking = motion_planner_config["docking"]

    if len(Entry_list)==0:
        return plan_path_l1(Motion_planner,robot_entry_,robot_target_,Entry_list,Target_list,current_joint_angles,
                            collision_info,pose_collision_detection,path_config,
                            mp_collision_detection_without_gui,libraries)
    else:
        return plan_path_mis(Motion_planner,robot_entry_,robot_target_,Entry_list,Target_list,current_joint_angles,
                            collision_info,pose_collision_detection,path_config,
                            mp_collision_detection_without_gui,libraries)
    
from utils_functions import *
libraries=[time,math,np,json,
		   scipyRotation,pybullet,pandas,
		   Plane,transforms3d,urllib3,o3d,go,t3d]


def plan_path_funapi(robot_entry_,robot_target_,collision_info,current_joint_angles,
              path_config,motion_planner_config,MOVE_PHYSICAL_ROBO):
    from MotionPlanner.MotionPlanner import Motion_planner
    from Robot_Configuration import RobotConfig
    from Pose.pose import Pose_Module
    from Collision.collision_detection import CollisionDetection
    from Collision.pose_collison_check import pose_collision_check
    Utils_functions = Utils_Functions(libraries)
    Robot_Config = RobotConfig(libraries,motion_planner_config,Utils_functions,MOVE_PHYSICAL_ROBO)
    Pose_module = Pose_Module(libraries,motion_planner_config,Utils_functions,
                 Robot_Config)
    MP_planner = Motion_planner(Robot_Config,Pose_module)
    pose_collision_detection = pose_collision_check(pybullet, pybullet.DIRECT, libraries, 
					bullet_client, Robot_Config,Robot_Config.get_rotary_joint_indices()[-1],
					collision_info,motion_planner_config,Utils_functions)
    mp_collision_detection_without_gui = CollisionDetection(pybullet, pybullet.DIRECT, libraries,
					bullet_client, Robot_Config,Robot_Config.get_rotary_joint_indices()[-1],
					collision_info,motion_planner_config,Utils_functions)
    path_flag,jointspace_list_ang_vel_time,needle_entry_list,needle_target_list,path_info_mis,final_pose_n_coll_flag=plan_path(MP_planner,													  
																			robot_entry_,robot_target_,collision_info,current_joint_angles,
																			path_config,pose_collision_detection,mp_collision_detection_without_gui,
																			libraries)
    datalike = {
             "path_flag":path_flag,
             "needle_entry_list":needle_entry_list,
             "needle_target_list":needle_target_list,
             "jointspace_list_ang_vel_time":jointspace_list_ang_vel_time,
             "final_pose_n_coll_flag":final_pose_n_coll_flag
					   }
    return datalike