import numpy as np

def plan(target_pos, current_pos, obstacles, target_distance, generate_num=6):
    if clear_path(target_pos, current_pos, obstacles):
        print('path clear between robot and ball')
        return target_pos
    #heading_options = np.random.random((generate_num,)) * np.pi * 2
    heading_options = np.linspace(0.0, 2*np.pi, num=generate_num)
    random_heading = np.random.random() * np.pi * 2
    best_target = current_pos + np.array([target_distance*np.cos(random_heading), target_distance*np.sin(random_heading)])
    for i in heading_options:
        #generate target_point at desired angle, distance
        potential_target = current_pos + np.array([target_distance*np.cos(i), target_distance*np.sin(i)])
        #see if there is a clear path between target point and current point
        #check to see if path gets closest to goal
        if clear_path(potential_target, current_pos, obstacles):
          #print('clear path between potential and current')
            if np.linalg.norm(potential_target - target_pos) < np.linalg.norm(best_target - target_pos):
                print('best target changed')
                best_target = potential_target
    return best_target

def clear_path(target_pos, current_pos, obstacles):
    AB = target_pos - current_pos
    # print('AB', AB)
    for o in obstacles:
        AC = o[:2] - current_pos
        # print('AC', AC)
        AD = AB * np.dot(AC, AB) / np.dot(AB, AB)
        # print('AD', AD)
        D = current_pos + AD
        # print('D', D)
        # print('distance between D and obstacle center', np.linalg.norm(D - o[:2]))
        if np.linalg.norm(D - o[:2]) <= o[2]:
            return False
    return True