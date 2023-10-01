import os
import numpy as np
import gymnasium as gym
import pickle
import matplotlib.pyplot as plt
import imageio
import random
from minigrid.core.world_object import Goal, Key, Door, Wall

MF = 0  # Move Forward
TL = 1  # Turn Left
TR = 2  # Turn Right
PK = 3  # Pickup Key
UD = 4  # Unlock Door


def motion_model(state_robot,control,env):

    '''
    input: x[0] = x_loc
           x[1] = y_loc
           x[2] = direction (4 possibilites)
           x[3] = door_status
           x[4] = key_status
    '''
    next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
    ## finding the next possible state after moving forward

    if state_robot[2] == 0: # Facing Right
        next_poss_state = [state_robot[0] + 1, state_robot[1], state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 1: # Facing down
        next_poss_state = [state_robot[0] , state_robot[1] + 1, state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 2: # Facing left
        next_poss_state = [state_robot[0] - 1 , state_robot[1], state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 3: # Facing up
        next_poss_state = [state_robot[0], state_robot[1] - 1, state_robot[2], state_robot[3], state_robot[4]]

    if control == MF:

        if isinstance(env.grid.get(next_poss_state[0],next_poss_state[1]),Wall):
            next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
        
        elif isinstance(env.grid.get(next_poss_state[0],next_poss_state[1]),Door):
            if state_robot[3]==1: ## door closed
                next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
            elif state_robot[3] ==0: ## door open
                next_state = tuple(next_poss_state)
        
        elif isinstance(env.grid.get(next_poss_state[0],next_poss_state[1]),Key):
            if state_robot[4]==0: ## not carrying key
                next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
            elif state_robot[4] == 1: ## carrying key
                next_state = tuple(next_poss_state)
            
        else:
            next_state = tuple(next_poss_state)

    elif control == TL:

        if state_robot[2] == 0: # Facing Right
            next_state = (state_robot[0], state_robot[1], 3, state_robot[3], state_robot[4])
        if state_robot[2] == 1: # Facing down
            next_state = (state_robot[0], state_robot[1], 0, state_robot[3], state_robot[4])
        if state_robot[2] == 2: # Facing left
            next_state = (state_robot[0], state_robot[1], 1, state_robot[3], state_robot[4])
        if state_robot[2] == 3: # Facing up
            next_state = (state_robot[0], state_robot[1], 2, state_robot[3], state_robot[4])
        
    elif control == TR:
        if state_robot[2] == 0: # Facing Right
            next_state = (state_robot[0], state_robot[1], 1, state_robot[3], state_robot[4])
        if state_robot[2] == 1: # Facing down
            next_state = (state_robot[0], state_robot[1], 2, state_robot[3], state_robot[4])
        if state_robot[2] == 2: # Facing left
            next_state = (state_robot[0], state_robot[1], 3, state_robot[3], state_robot[4])
        if state_robot[2] == 3: # Facing up
            next_state = (state_robot[0], state_robot[1], 0, state_robot[3], state_robot[4])
    
    elif control == UD:

        if isinstance(env.grid.get(next_poss_state[0],next_poss_state[1]),Door):
            if state_robot[4] == 1: ## has key
                next_state = (state_robot[0], state_robot[1], state_robot[2], 0, state_robot[4])
            else:
                next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
    
    elif control == PK:

        if isinstance(env.grid.get(next_poss_state[0],next_poss_state[1]),Key) and state_robot[4] ==0:
            next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], 1)
        else:
            next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
    
    else:
        next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
        

    return next_state



def step_cost(state_robot,control_input,env):
    # You should implement the stage cost by yourself
    # Feel free to use it or not
    # ************************************************
    
    # next_state = motion_model(state_robot,control_input,env)

    next_state = (state_robot[0], state_robot[1], state_robot[2], state_robot[3], state_robot[4])
    ## finding the next possible state after moving forward

    if state_robot[2] == 0: # Facing Right
        next_state = [state_robot[0] + 1, state_robot[1], state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 1: # Facing down
        next_state = [state_robot[0] , state_robot[1] + 1, state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 2: # Facing left
        next_state = [state_robot[0] - 1 , state_robot[1], state_robot[2], state_robot[3], state_robot[4]]
    if state_robot[2] == 3: # Facing up
        next_state = [state_robot[0], state_robot[1] - 1, state_robot[2], state_robot[3], state_robot[4]]


    stage_cost = 1

    if control_input == MF:

        if isinstance(env.grid.get(next_state[0],next_state[1]),Wall):
            stage_cost = np.inf
    
        if isinstance(env.grid.get(next_state[0],next_state[1]),Door) and state_robot[3]==1: # door closed
            stage_cost = np.inf
        
        if isinstance(env.grid.get(next_state[0],next_state[1]),Key) and state_robot[4]==0: # not carrying key
            stage_cost = np.inf
    
    elif control_input == UD:
        if isinstance(env.grid.get(next_state[0],next_state[1]),Door) and state_robot[4] == 1:
            if state_robot[3]==1: # door closed
                stage_cost = 1
                
            else: # door open
                stage_cost = np.inf
        else:
            stage_cost = np.inf


    elif control_input == PK:
        if isinstance(env.grid.get(next_state[0],next_state[1]),Key) and state_robot[4]==0: # not carrying key
            if state_robot[3] ==1:
                stage_cost = 1
            else:
                stage_cost = np.inf
        else:
            stage_cost = np.inf
            
            
    elif control_input == TL or control_input ==TR:
        stage_cost = 1

    return stage_cost


def step(env, action):
    """
    Take Action
    ----------------------------------
    actions:
        0 # Move forward (MF)
        1 # Turn left (TL)
        2 # Turn right (TR)
        3 # Pickup the key (PK)
        4 # Unlock the door (UD)
    """
    actions = {
        0: env.actions.forward,
        1: env.actions.left,
        2: env.actions.right,
        3: env.actions.pickup,
        4: env.actions.toggle,
    }

    (obs, reward, terminated, truncated, info) = env.step(actions[action])
    return obs, terminated


def generate_random_env(seed, task):
    """
    Generate a random environment for testing
    -----------------------------------------
    seed:
        A Positive Integer,
        the same seed always produces the same environment
    task:
        'MiniGrid-DoorKey-5x5-v0'
        'MiniGrid-DoorKey-6x6-v0'
        'MiniGrid-DoorKey-8x8-v0'
    """
    if seed < 0:
        seed = np.random.randint(50)
    env = gym.make(task, render_mode="rgb_array")
    env.reset(seed=seed)
    return env


def load_env(path):
    """
    Load Environments
    ---------------------------------------------
    Returns:
        gym-environment, info
    """
    with open(path, "rb") as f:
        env = pickle.load(f)

    info = {"height": env.height, "width": env.width, "init_agent_pos": env.agent_pos, "init_agent_dir": env.dir_vec, "wall_pos": []}

    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i), Key):
                info["key_pos"] = np.array([j, i])
            elif isinstance(env.grid.get(j, i), Door):
                info["door_pos"] = np.array([j, i])
            elif isinstance(env.grid.get(j, i), Goal):
                info["goal_pos"] = np.array([j, i])
            elif isinstance(env.grid.get(j, i), Wall):
                info["wall_pos"].append(np.array([j, i]))

    return env, info


def load_random_env(env_folder):
    """
    Load a random DoorKey environment
    ---------------------------------------------
    Returns:
        gym-environment, info
    """
    env_list = [os.path.join(env_folder, env_file) for env_file in os.listdir(env_folder)]
    env_path = random.choice(env_list)
    with open(env_path, "rb") as f:
        env = pickle.load(f)

    info = {
        "height": env.height,
        "width": env.width,
        "init_agent_pos": env.agent_pos,
        "init_agent_dir": env.dir_vec,
        "door_pos": [],
        "door_open": [],
        "wall_pos": []
    }

    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i), Key):
                info["key_pos"] = np.array([j, i])
            elif isinstance(env.grid.get(j, i), Door):
                info["door_pos"].append(np.array([j, i]))
                if env.grid.get(j, i).is_open:
                    info["door_open"].append(True)
                else:
                    info["door_open"].append(False)
            elif isinstance(env.grid.get(j, i), Goal):
                info["goal_pos"] = np.array([j, i])
            elif isinstance(env.grid.get(j, i), Wall):
                info["wall_pos"].append(np.array([j, i]))

    return env, info, env_path


def save_env(env, path):
    with open(path, "wb") as f:
        pickle.dump(env, f)


def plot_env(env):
    """
    Plot current environment
    ----------------------------------
    """
    img = env.render()
    plt.figure()
    plt.imshow(img)
    plt.show()


def draw_gif_from_seq(seq, env, path="./gif/doorkey.gif"):
    """
    Save gif with a given action sequence
    ----------------------------------------
    seq:
        Action sequence, e.g [0,0,0,0] or [MF, MF, MF, MF]

    env:
        The doorkey environment
    """
    with imageio.get_writer(path, mode="I", duration=0.8) as writer:
        img = env.render()
        writer.append_data(img)
        for act in seq:
            img = env.render()
            step(env, act)
            writer.append_data(img)
    print(f"GIF is written to {path}")
    return
