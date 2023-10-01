from utils_parta_ready import *
# from example import example_use_of_gym_env
import itertools
import numpy as np
from tqdm import tqdm
from pdb import Pdb
import csv
import pickle
from itertools import product
from copy import deepcopy

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


### SPECIFY THE PATH OF THE KNOWN ENVIRONMENT TO LOAD THE ENVIRONMENT ###

path_env = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\ECE276B_PR1\\starter_code\\envs\\known_envs\\doorkey-8x8-normal.env"
path_gif = "D:\\UCSD Spring 2023\\ECE 276B\\ECE276B_PR1\\276B_PR1_CODE_final\\partA\\gif_partA\\doorkey-8x8-normal.gif"


def doorkey_problem(env,info):
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
    ## finding the size of the grid
    ## defining the stage cost matrix (shape )

    map_rows = info['height']
    map_cols = info['width']
    map_size = map_rows * map_cols

    # Get Key status
    key_pos = info["key_pos"]

    door_pos = info['door_pos']

    goal_pos = info["goal_pos"]

    wall_pos = info["wall_pos"]

    # Get the door status
    door = env.grid.get(info["door_pos"][0], info["door_pos"][1])
    is_open = door.is_open
    is_locked = door.is_locked

    ###  DEFINING THE STATE SPACE ###

    state_space = {}
  
    state_index = 0
    for x in range(env.width):
        for y in range(env.height):

            if isinstance(env.grid.get(x,y),Wall):  ## not considering the walls in the state space
                continue
            else:
                for orientation, door_status, key_status in product(range(4), range(2), range(2)):
                    state_space[(x,y,orientation,door_status,key_status)] = state_index
                    state_index += 1
   

    ### DEFINING THE PLANNING HORIZON ###
    
    T = len(state_space) - 1 # (Time horizon)


    ### DEFINING THE TERMINAL COST AND ASSIGNING IT TO THE VALUE FUNCTION AT THE TERMINAL STATE ###
 
    Q_func = np.ones([len(state_space),5])

    Value_func = np.ones([T,len(state_space)])*np.inf

    x_goal, y_goal = goal_pos[0], goal_pos[1]

    for orientation, door_status, key_status in product(range(4), range(2), range(2)):

        goal_state = (x_goal,y_goal,orientation,door_status,key_status)
        goal_idx = state_space[goal_state]
        Value_func[T-1,goal_idx] = -100
        Q_func[goal_idx] = -100

    
    optimal_control = np.ones([T,len(state_space)])

       
   ### IMPLEMENTING DYNAMIC PROGRAMMING ###

    for t in tqdm(range(T-2,-1,-1)):
        for x in range(env.width):
            for y in range(env.height):

                if isinstance(env.grid.get(x,y),Wall):  ## not considering the walls in the state space
                    continue
                elif isinstance(env.grid.get(x,y),Goal):  ## not considering the walls in the state space
                    continue
                else:
                    for orientation, door_status, key_status, control_input in product(range(4), range(2), range(2),range(5)):
                        curr_state =(x,y,orientation,door_status,key_status)
                        curr_state_idx = state_space[curr_state]
                        next_state = motion_model(curr_state,control_input,env)
                        next_state_idx = state_space[tuple(next_state)]
                        cost = step_cost(curr_state,control_input,env)
                        Q_func[curr_state_idx,control_input] = cost + Value_func[t+1,next_state_idx]
        
        Value_func[t] = np.min(Q_func,axis=1)
        optimal_control[t] = np.argmin(Q_func,axis=1)

        if np.array_equal(Value_func[t], Value_func[t+1]):
            t_start = t
            break
    

    return optimal_control,Value_func,t_start,state_space




def partA():
 
    env,info = load_env(path_env)
    opt_control,val_func,t_start,state_space = doorkey_problem(env,info)  # find the optimal action sequence

    print(t_start)
    door = env.grid.get(info["door_pos"][0], info["door_pos"][1])
    is_locked = door.is_locked
    door_status_init = int(is_locked)


    x_start = env.agent_pos[0]
    y_start = env.agent_pos[1]
    orient_start = directions[tuple(env.dir_vec)]
    is_carrying = env.carrying is not None
    key_status_init =int(is_carrying)
    
    agent_state_init = (x_start,y_start,orient_start,door_status_init,key_status_init)
    agent_state_idx_init = state_space[agent_state_init]

    agent_state = deepcopy(agent_state_init)
    agent_state_idx = deepcopy(agent_state_idx_init)

    t=t_start
    opt_seq = []
    # print(tuple(info['goal_pos']))
    
    while (agent_state[0],agent_state[1]) != tuple(info['goal_pos']):

        action = opt_control[t,agent_state_idx]
        step(env, int(action))
        opt_seq.append(int(action))
        x = env.agent_pos[0]
        y  = env.agent_pos[1]
        orient = directions[tuple(env.dir_vec)]
        is_carrying = env.carrying is not None
        key_status =int(is_carrying)
        is_locked = door.is_locked
        door_status = int(is_locked)
        agent_state = (x,y,orient,door_status,key_status)
        agent_state_idx = state_space[agent_state]
        t=t+1


    opt_seq.append(0)
    print("optimal action sequence : ",opt_seq)

    return opt_seq

    



def partB():
    env_folder = "./envs/random_envs"
    env, info, env_path = load_random_env(env_folder)


if __name__ == "__main__":
    # example_use_of_gym_env()
    opt_seq = partA()
    env,info = load_env(path_env)
    draw_gif_from_seq(opt_seq,env, path_gif)

    # partB()
