import numpy as np

#In-house real-time path planner inspired by RRT and A*, modified to work with moving obstacles

def plan(goal_pos, current_pos, obstacles, target_distance, generate_num=6):

    if np.linalg.norm(goal_pos - current_pos) < 5:
        return current_pos

    #If no obstacles between current position and goal, return goal as target
    if exists_clear_path(goal_pos, current_pos, obstacles):
        return goal_pos

    #Random heading generation
    #heading_options = np.random.random((generate_num,)) * np.pi * 2
    #Evenly spaced heading generation
    heading_options = np.linspace(0.0, 2*np.pi, num=generate_num)

    #Set default target to random, in case of obstacle collision
    random_heading = np.random.random() * np.pi * 2
    best_target = current_pos + np.array([target_distance*np.cos(random_heading), target_distance*np.sin(random_heading)])

    #For each heading option, generate a potential target
    #If potential target does not route though any obstacles and is closer to goal than current best_target, update best_target to potential_target
    for i in heading_options:
        potential_target = current_pos + np.array([target_distance*np.cos(i), target_distance*np.sin(i)])
        if exists_clear_path(potential_target, current_pos, obstacles):
            if np.linalg.norm(potential_target - goal_pos) < np.linalg.norm(best_target - goal_pos):
                best_target = potential_target

    return best_target


#Determins if path between current_pos and goal_pos is clear using vector projection algorithm

def exists_clear_path(goal_pos, current_pos, obstacles):

    AB = goal_pos - current_pos
    for o in obstacles:
        AC = o[:2] - current_pos
        AD = AB * np.dot(AC, AB) / np.dot(AB, AB)
        D = current_pos + AD
        if np.linalg.norm(D - o[:2]) <= o[2]:
            return False
    return True