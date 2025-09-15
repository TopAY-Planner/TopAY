import numpy as np 
class MomaParam: 
    chassis_length = 0.685
    chassis_width = 0.57
    chassis_height = 0.155
    #! TEST ZMK
    # max_v = 3.0
    # max_a = 2.0
    # max_w = 4.0
    # max_dw = 6.0
    #! TEST ZMK
    max_v = 1.0
    max_a = 0.8
    max_w = 0.9
    max_dw = 1.0
    dof_num = 7
    link_length = np.array([0.2405, 0.0, 0.256, 0.0, 0.21, 0.0, 0.144])
    joint_pos_limit_min = [-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28]
    joint_pos_limit_max = [3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28]
    joint_vel_limit = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
    joint_acc_limit  = list(np.array(joint_vel_limit) * 1.5)
    joint_eff_limit = [60.0, 60.0, 30.0, 30.0, 10.0, 10.0, 10.0]
    joint_offset = np.array([[-1.5708, 0.0, 0.0],
                            [1.5708, 0.0, 0.0],
                            [-1.5708, 0.0, 0.0],
                            [1.5708, 0.0, 0.0],
                            [-1.5708, 0.0, 0.0],
                            [1.5708, 0.0, 0.0],
                            [0.0, 0.0, 0.0]])
    joint_dof_axis = np.array([[0.0, -1.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]])

