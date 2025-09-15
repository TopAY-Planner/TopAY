#! /usr/bin/env python3
import time
import rospy
import casadi as ca
from casadi import sin, cos, pi
import numpy as np
import sys
import math
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry
import os
from moma_param import MomaParam
from fake_moma.msg import MomaCmd
from fake_moma.msg import MomaState

def BuildSXMatrix(matrix_list: list):
    """
    直接用SX(list)无法生成带符号量的matrix/vector, 可以使用逻辑判断表达式等
    Args:
        matrix_list: 包含sx符号量的二维list格式
    Returns: sx矩阵
    """
    assert isinstance(matrix_list, list)
    if not isinstance(matrix_list[0], list):  # 如果不是list就说明是向量
        rows = len(matrix_list)
        sx_matrix = ca.SX.zeros(rows, 1)
        for i in range(rows):
            sx_matrix[i, 0] = matrix_list[i]
        return sx_matrix
    rows, cols = len(matrix_list), len(matrix_list[0])
    sx_matrix = ca.SX.zeros(rows, cols)
    # print(sx_matrix)
    for i in range(rows):
        for j in range(cols):
            sx_matrix[i, j] = matrix_list[i][j]
    return sx_matrix

def AxisAngToQuaternion(omega, theta):
    """
    将轴角转换为四元数
    :param omega: 旋转矢量
    :param theta: 旋转角度
    :return: 四元数(x,y,z,w)
    """
    q = ca.vertcat(omega * ca.sin(theta / 2), ca.cos(theta / 2))
    return q

def MatrixLog3(R):
    """
    旋转矩阵映射到旋转向量(角轴)，即对数映射。
    :param R: 旋转矩阵
    :return:  角轴
    """
    # assert isinstance(R, ca.SX)
    theta = ca.arccos((ca.trace(R) - 1) / 2)
    omega = ca.vertcat(R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]) / (2 * ca.sin(theta))
    return omega, theta

def RotationToQuaternion(R):
    """
    旋转矩阵变换为四元数。
    :param R: 旋转矩阵
    :return: 四元数(x,y,z,w)
    """
    omega, theta = MatrixLog3(R)
    q = AxisAngToQuaternion(omega, theta)
    return q

def VecToso3(omg):
    # 不用验证omg类型
    so3mat_list = [[0, -omg[2], omg[1]],
                   [omg[2], 0, -omg[0]],
                   [-omg[1], omg[0], 0]]
    so3mat = BuildSXMatrix(so3mat_list)
    return so3mat

def QuaternionError(q1, q2):
    """
    计算四元数之间的误差。
    :param q1: 四元数1
    :param q2: 四元数2
    :return: 误差
    """
    tmp = VecToso3(q2)
    return q1[3] * q2[:3] - q2[3] * q1[:3] + tmp @ q1[:3]


def RotationErrorByQuaternion(R1, R2):
    """
    通过四元数计算旋转矩阵之间的误差。
    :param R1: 旋转矩阵1
    :param R2: 旋转矩阵2
    :return: 误差
    """
    q1 = RotationToQuaternion(R1)
    q2 = RotationToQuaternion(R2)
    return QuaternionError(q1,q2)

def RobFki(st):
    phi = st[2]
    p_ee = ca.vertcat(st[0], st[1], MomaParam.chassis_height)
    R_ee = ca.vertcat(
        ca.horzcat(ca.cos(phi), -ca.sin(phi), 0),
        ca.horzcat(ca.sin(phi), ca.cos(phi), 0),
        ca.horzcat(0, 0, 1)
    )
    q = st[3:]
    for i in range(6):
        p_ee += R_ee @ ca.vertcat(0, 0, MomaParam.link_length[i])
        alpha = MomaParam.joint_offset[i][0]
        Rxa = ca.vertcat(
                ca.horzcat(1, 0, 0),
                ca.horzcat(0, ca.cos(alpha), -ca.sin(alpha)),
                ca.horzcat(0, ca.sin(alpha), ca.cos(alpha)),
            )
        zeta = q[i] * MomaParam.joint_dof_axis[i][1]
        Ryz = ca.vertcat(
                ca.horzcat(ca.cos(zeta), 0, ca.sin(zeta)),
                ca.horzcat(0, 1, 0),
                ca.horzcat(-ca.sin(zeta), 0, ca.cos(zeta))
            )
        R_ee = R_ee @ Rxa @ Ryz
    p_ee += R_ee @ ca.vertcat(0, 0, MomaParam.link_length[-1])
    zeta = q[-1] * MomaParam.joint_dof_axis[-1][2]
    R_ee = R_ee @ ca.vertcat(
                ca.horzcat(ca.cos(zeta), -ca.sin(zeta), 0),
                ca.horzcat(ca.sin(zeta), ca.cos(zeta), 0),
                ca.horzcat(0, 0, 1)
            )
        
    return p_ee, R_ee

class MPC:
    def __init__(self):
        self.rcv_odom = False
        
        # set the Parameters of MPC 
        self.Q = ca.diagcat(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        Rv, Rw, Ra = 1, 1, 1
        self.R = ca.diagcat(Rv, Rw, Ra, Ra, Ra, Ra, Ra, Ra, Ra)
        self.step_horizon = 0.1  # time between steps in seconds
        self.N = 30  # number of look ahead steps
        self.ctrl_fre_t = 0.02 

        # states symbolic variables
        self.n_states = 10
        # control symbolic variables
        self.n_controls = 9
        
        # set the para of arm
        self.Qa = ca.diag([200, 200, 200])
        self.Qa_rot = ca.diag([10, 10, 10])
        
        # initial state
        self.state_init = ca.DM(np.zeros(self.n_states).tolist())  # initial state
        # 末端执行器的目标位姿
        self.target_se3 = ca.DM.zeros(7 * (self.N + 1))
        pose_init = [0,0,1.0,0,0.0,0,1.0] # [x,y,z, ox,oy,oz,ow]
        for k in range(self.N + 1):
            for j in range(7):
                self.target_se3[k * 7 + j] = pose_init[j]
        
        # optimization variables
        self.u0 = ca.DM.zeros((self.n_controls, self.N))  # initial control
        self.X0 = ca.repmat(self.state_init, 1, self.N + 1)  # initial state full

        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()
        
        # rospy
        rospy.init_node("mpc_node")
        rospy.Subscriber("/manual_target", Pose, self.target_pose_sub)
        # rospy.Subscriber("/moma_target", Pose, self.target_pose_sub)
        rospy.Subscriber("/fake_moma_node/state", MomaState, self.state_sub)
        self.cmd_pub = rospy.Publisher('/moma_cmd', MomaCmd, queue_size=50)
        self.predict_pub_base = rospy.Publisher('/predict_path_base', Path, queue_size=50)
        self.predict_pub_ee = rospy.Publisher('/predict_path_ee', Path, queue_size=50)
        rospy.Timer(rospy.Duration(self.ctrl_fre_t), self.control_cb)
        rospy.spin()

    def create_symbolic_variables(self):
        # state symbolic variables
        q_mm = ca.SX.sym('q_mm', self.n_states)  # (x,y,theta,q1-q7)
        u_mm = ca.SX.sym('u_mm', self.n_controls)  # dot(v,w,q1-q7)

        # matrix containing all states over all time steps +1 (each column is a state vector)
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)
        # matrix containing all control actions over all time steps (each column is an action vector)
        self.U = ca.SX.sym('U', self.n_controls, self.N)
        self.P = ca.SX.sym('P', self.n_states + 7 * (self.N + 1))
        self.P_arm = ca.reshape(self.P[self.n_states:], 7, self.N + 1)
        theta = q_mm[2]
        v = u_mm[0]
        omega = u_mm[1]
        RHS = ca.vertcat(v * ca.cos(theta),
                        v * ca.sin(theta),
                        omega,
                        u_mm[2],
                        u_mm[3],
                        u_mm[4],
                        u_mm[5],
                        u_mm[6],
                        u_mm[7],
                        u_mm[8])
        self.f = ca.Function('f_mm', [q_mm, u_mm], [RHS])

    def build_cost_function(self):
        self.cost_fn = 0  # cost function
        self.g = self.X[:, 0] - self.P[:self.n_states]  # constraints in the equation
        # runge kutta
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            x_arm, R_arm = RobFki(st)
            qua_arm = RotationToQuaternion(R_arm)
            r_err = QuaternionError(qua_arm, self.P_arm[3:, k])
            self.cost_fn = self.cost_fn \
                        + st.T @ self.Q @ st \
                        + con.T @ self.R @ con \
                        + (x_arm - self.P_arm[:3, k]).T @ self.Qa @ (x_arm - self.P_arm[:3, k]) \
                        + r_err.T @ self.Qa_rot @ r_err 
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_RK4 = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, st_next - st_next_RK4)

    def set_optimize_option(self):
        OPT_variables = ca.vertcat(
            self.X.reshape((-1, 1)),  # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            self.U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': self.cost_fn,
            'x': OPT_variables,
            'g': self.g,
            'p': self.P
        }
        opts = {
            'ipopt': {
                'max_iter': 1000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_limit(self):
        n_states, n_controls = self.n_states, self.n_controls
        N = self.N
        lbx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))
        ubx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))

        for i in range(3):
            lbx[i: n_states * (N + 1): n_states] = -ca.inf  # XYtheta lower bound
            ubx[i: n_states * (N + 1): n_states] = ca.inf   # XYtheta upper bound
            
        for i in range(7):
            lbx[i+3: n_states * (N + 1): n_states] = MomaParam.joint_pos_limit_min[i]  # q1 lower bound
            ubx[i+3: n_states * (N + 1): n_states] = MomaParam.joint_pos_limit_max[i]  # q1 lower bound
            
        lbx[n_states * (N + 1): -1: n_controls] = -MomaParam.max_v  # u upper bound for v
        lbx[n_states * (N + 1) + 1: -1: n_controls] = -MomaParam.max_w  # u upper bound for w
        ubx[n_states * (N + 1): -1: n_controls] = MomaParam.max_v  # u upper bound for v
        ubx[n_states * (N + 1) + 1: -1: n_controls] = MomaParam.max_w  # u upper bound for w
        for i in range(7):
            lbx[n_states * (N + 1)+i+2: -1: n_controls] = -MomaParam.joint_vel_limit[i]  # u lower bound for q1
            ubx[n_states * (N + 1)+i+2: -1: n_controls] = MomaParam.joint_vel_limit[i]  # u lower bound for q1

        self.args = {
            'lbg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints lower bound
            'ubg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        
    def target_pose_sub(self, pose_msg):
        # print(f"Target:\n{pose_msg}")
        p, ori = pose_msg.position, pose_msg.orientation
        args = [p.x, p.y, p.z, ori.x, ori.y, ori.z, ori.w]
        for k in range(self.N + 1):
            for j in range(7):
                self.target_se3[k * 7 + j] = args[j]

    def state_sub(self, moma_state_msg):
        p = moma_state_msg.chassis_odom.pose.pose.position
        ori = moma_state_msg.chassis_odom.pose.pose.orientation
        theta = math.atan2(2.0*ori.z*ori.w, 2.0*ori.w**2-1.0)
        state_list = [p.x, p.y, theta, 0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            state_list[3+i] = moma_state_msg.arm_odom[i].twist.twist.linear.x
        self.state_init = ca.DM(state_list)
        self.rcv_odom = True
    
    def control_cb(self, _):
        if self.rcv_odom:
            t_start = time.time()
            p = ca.vertcat(self.state_init, self.target_se3)
            self.args['p'] = p
            self.args['x0'] = ca.vertcat(
                    ca.reshape(self.X0, self.n_states * (self.N + 1), 1),
                    ca.reshape(self.u0, self.n_controls * self.N, 1)
                )
            sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )
            self.u0 = ca.reshape(sol['x'][self.n_states * (self.N + 1):], self.n_controls, self.N)
            self.X0 = ca.reshape(sol['x'][: self.n_states * (self.N + 1)], self.n_states, self.N + 1)
            cmd_msg = MomaCmd()
            cmd_msg.speed = self.u0[0, 0]
            cmd_msg.angular_velocity = self.u0[1, 0]
            cmd_msg.q.data = [0, 0, 0, 0, 0, 0, 0]
            cmd_msg.dq.data = [0, 0, 0, 0, 0, 0, 0]
            for i in range(7):
                cmd_msg.q.data[i] = self.state_init[3+i] + self.ctrl_fre_t * self.u0[2+i, 0]
                cmd_msg.dq.data[i] = self.u0[2+i, 0]
            self.cmd_pub.publish(cmd_msg)
            print((time.time() - t_start)*1000.0, "ms")
            
if __name__=='__main__':
    try:
        mpc = MPC()
    except:
        rospy.logwarn("cannot start MPC")
