from utils_partb_ready import *
# from example import example_use_of_gym_env
import itertools
import numpy as np
from tqdm import tqdm
from pdb import Pdb
import csv
import pickle
from itertools import product
from copy import deepcopy
from collections import defaultdict

MF = 0  # Move Forward
TL = 1  # Turn Left
TR = 2  # Turn Right
PK = 3  # Pickup Key
UD = 4  # Unlock Door

directions ={
    (1,0) :0,
    (0,1) :1,
    (-1,0):2,
    (0,-1):3
}

key_locations = [(1,1),(2,3),(1,6)]
door_locations = [(4,2),(4,5)]
wall_locations = [(4,0),(4,1),(4,3),(4,4),(4,6),(4,7)]
goal_locations = [(5,1), (6,3),(5,6)]

### SPECIFY THE PATH OF THE RANDOM ENVIRONMENT TO LOAD THE ENVIRONMENT ###

i=1
path_env = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\ECE276B_PR1\\starter_code\\envs\\random_envs\\DoorKey-8x8-{}.env".format(i)
path_gif = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\ECE276B_PR1\\starter_code\\envs\\random_envs\\DoorKey-8x8-{}.gif".format(i)


def doorkey_problem():
    """
    You are required to find the optimal path in
        doorkey-5x5-normal.env
        doorkey-6x6-normal.env
        doorkey-8x8-normal.env

        doorkey-6x6-direct.env
        doorkey-8x8-direct.env

        doorkey-6x6-shortcut.env
        doorkey-8x8-shortcut.env

    Feel Free to modify this fuction
    """
    

    ###  DEFINING THE STATE SPACE ###

    state_space = {}

    map_rows = 8
    map_cols = 8
    map_size = map_rows * map_cols
 

    state_index = 0
    for x in range(map_cols):
        for y in range(map_rows):
            if (x,y) in wall_locations:  ## not considering the walls in the state space
                continue
            else:
                for orientation, key_pos, goal_pos, door1_status, door2_status,key_status in product(range(4), range(3), range(3),range(2), range(2), range(2)):
                    state_space[(x,y,orientation,key_pos,goal_pos,door1_status,door2_status,key_status)] = state_index
                    state_index += 1
   
  
   ### DEFINING THE PLANNING HORIZON ###

    T = len(state_space) - 1 # Planning Horizon


   ### DEFINING THE TERMINAL COST AND ASSIGNING IT TO THE VALUE FUNCTION AT THE TERMINAL STATE ###

    Q_func = np.ones([len(state_space),5])

    Value_func = np.ones([T,len(state_space)])*np.inf

    for goal_pos in range(3):

        x_goal = goal_locations[goal_pos][0]
        y_goal = goal_locations[goal_pos][1]

        for orientation, key_pos, door1_status, door2_status, key_status in product(range(4), range(3), range(2), range(2), range(2)):

            goal_state = (x_goal, y_goal, orientation, key_pos, goal_pos, door1_status, door2_status, key_status)
            goal_idx = state_space[goal_state]
            Value_func[T-1,goal_idx] = -100
            Q_func[goal_idx] = -100

   
    optimal_control = np.ones([T,len(state_space)])


    ### IMPLEMENTING DYNAMIC PROGRAMMING ###

    for t in tqdm(range(T-2,-1,-1)):
        for goal_pos in range(3):
       
            for x in range(map_cols):
                for y in range(map_rows):
                    
                        if (x,y) in wall_locations:  ## not considering the walls in the state space
                            continue
                        elif (x,y) == goal_locations[goal_pos]:  ## not considering the goal position in the DP as it is the terminal state
                            continue
                        else:
                            for orientation, key_pos, door1_status, door2_status, key_status, control_input in product(range(4), range(3), range(2), range(2), range(2),range(5)):
                                curr_state = (x, y, orientation, key_pos, goal_pos, door1_status, door2_status, key_status)
                                curr_state_idx = state_space[curr_state]

                                try:
                                    next_state = motion_model(curr_state,control_input,wall_locations,key_locations,door_locations)
                                    next_state_idx = state_space[tuple(next_state)]
                                    cost = step_cost(curr_state,control_input,wall_locations,door_locations,key_locations)
                                    Q_func[curr_state_idx,control_input] = cost + Value_func[t+1,next_state_idx]

                                except KeyError:
                                    next_state = curr_state
                                    next_state_idx = curr_state_idx
                                    Q_func[curr_state_idx,control_input] = np.inf + Value_func[t+1,next_state_idx]
                
        Value_func[t] = np.min(Q_func,axis=1)
        optimal_control[t] = np.argmin(Q_func,axis=1)  ## FINDING THE OPTIMAL CONTROL FOR ALL THE STATES AT TIME t


        if np.array_equal(Value_func[t], Value_func[t+1]): ## CHECKING FOR CONVERGENCE OF VALUE FUNCTION ##
            t_start = t
            break

    return optimal_control,Value_func,t_start,state_space

def partB():
    opt_seq = defaultdict(list)

    # for i in range(1,37):

    # i=6
    path_env = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\ECE276B_PR1\\starter_code\\envs\\random_envs\\DoorKey-8x8-{}.env".format(i)


    env, info = load_random_env(path_env)

    optimal_control, Value_function,t_start,state_space = doorkey_problem()

    x_start = env.agent_pos[0]
    y_start = env.agent_pos[1]
    orient_start = directions[tuple(env.dir_vec)]
    is_carrying = env.carrying is not None
    key_status_init =int(is_carrying)

    door1 = env.grid.get((info["door_pos"][0])[0], (info["door_pos"][0])[1])
    is_open_D1 = door1.is_open
    door1_status_init = int(is_open_D1)

    door2 = env.grid.get((info["door_pos"][1])[0], (info["door_pos"][1])[1])
    is_open_D2 = door2.is_open
    door2_status_init = int(is_open_D2)

    key_loc = info['key_pos']

    if tuple(key_loc) == key_locations[0]:
        key_pos = 0
    elif tuple(key_loc) == key_locations[1]:
        key_pos = 1
    elif tuple(key_loc) == key_locations[2]:
        key_pos = 2  

    

    goal_loc = info['goal_pos']

    if tuple(goal_loc) == goal_locations[0]:
        goal_pos = 0
    elif tuple(goal_loc) == goal_locations[1]:
        goal_pos = 1
    elif tuple(goal_loc) == goal_locations[2]:
        goal_pos = 2  


    agent_state_init = (x_start, y_start, orient_start, key_pos, goal_pos, door1_status_init, door2_status_init, key_status_init)

    agent_state_index_init = state_space[agent_state_init]

    T = len(state_space) - 1
    agent_state = deepcopy(agent_state_init)
    agent_state_index = deepcopy(agent_state_index_init)

    try: 

        t = t_start

        while (agent_state[0],agent_state[1]) != tuple(goal_loc):
            action = optimal_control[t,agent_state_index]
            control = int(action)
    
            opt_seq[i].append(control)
            
            step(env, control)
            
            x = env.agent_pos[0]
            y = env.agent_pos[1]
            orient = directions[tuple(env.dir_vec)]
            is_carrying = env.carrying is not None
            key_status =int(is_carrying)

            is_open_D1 = door1.is_open
            door1_status = int(is_open_D1)

            is_open_D2 = door2.is_open
            door2_status = int(is_open_D2)

        
            agent_state = (x, y, orient, key_pos, goal_pos, door1_status, door2_status, key_status)
        
            agent_state_index = state_space[agent_state]
            t = t + 1

        opt_seq[i].append(0)
        print("opt_seq_env:{} : ".format(i),opt_seq[i])

    except IndexError:
        print("opt_seq_env:{} : ".format(i),opt_seq[i])
        pass

    return Value_function, optimal_control, agent_state_index_init, opt_seq
   
    # draw_gif_from_seq(opt_seq[i],env,path_gif)

if __name__ == "__main__":
    
    Val_func, optimal_control, agent_index, opt_seq= partB()

    # for i in range(1,37):
    path_env = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\ECE276B_PR1\\starter_code\\envs\\random_envs\\DoorKey-8x8-{}.env".format(i)
    path_gif = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\276B_PR1_CODE_final\\partB\\gif_partb\\DoorKey-8x8-{}.gif".format(i)
# partB()

    env, info = load_random_env(path_env)
    # print(opt_seq[i])

    draw_gif_from_seq(opt_seq[i],env,path_gif)
