import numpy as np

def transform(t_global_x, t_global_y, r_global_x, r_global_y, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    x = t_global_x - r_global_x
    y = t_global_y - r_global_y
    t_robot_x = c*x + s*y
    t_robot_y = -s*x + c*y
    
    return [t_robot_x, t_robot_y]