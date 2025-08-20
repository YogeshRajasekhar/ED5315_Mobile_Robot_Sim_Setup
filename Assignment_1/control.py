import robot_params
import numpy as np 
import time

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = time.time()

def at_goal(robot_state, goal_state):    
    
    #check if we have reached goal point
    d = np.sqrt((goal_state[0]-robot_state[0])**2 + (goal_state[1]-robot_state[1])**2)
    
    if d <= robot_params.goal_threshold:
        return True
    else:
        return False

def gtg(robot_state, goal_state):  
    #The Go to goal controller
    
    global prev_heading_error
    global total_heading_error  
    global previous_time
    
    #Controller parameters
    Kp = 0.00656
    Kd = 0.0001
    K = 0.4
    
    #determine how far to rotate to face the goal point
    e_new = goal_state[-1]-robot_state[-1]
    e_new_degree = np.degree(e_new)
    
    #Remember to restrict error to (-180 ,180)
    e_new_degree_res = (e_new_degree+180) % 360 - 180
    
    current_time = time.time()
    dt = current_time-previous_time
    previous_time = current_time
    
    #PD controller for angular velocity
    e_dot = (e_new_degree_res - prev_heading_error)/dt
    W = Kp*e_new_degree_res + Kd*e_dot
    prev_heading_error = e_new_degree_res

    #find distance to goal
    d = np.sqrt((goal_state[0]-robot_state[0])**2 + (goal_state[1]-robot_state[1])**2)
    
    #P control for linear velocity
    V = K*d
    
    #request robot to execute velocity
    return[V,W]


