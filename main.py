import numpy as np
import matplotlib.pyplot as plt
import cvxpy
import sys
import pathlib

#状态数
NX,NU,T=4,2,5

#MPC 参数
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramters
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param
TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number
DT = 0.2  # 小T

# 车辆信息
LENGTH = 4.5
WIDTH = 2.0
BACKTOWHEEL = 1.0
WHEEL_LEN = 0.3
WHEEL_WIDTH = 0.2
TREAD = 0.7
WB = 2.5  #前轮到后轴的距离
MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  #最大速度
MIN_SPEED = -20.0 / 3.6  #最小速度
MAX_ACCEL = 1.0  #最大加速度
show_animation = True

MAX_STEER = np.deg2rad(45.0)
MIN_STEER = -MAX_STEER


class state:
    """
    vehicle state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

    def pi_2_pi(angle):
        if angle > np.pi:
            angle = angle - 2.0 * np.pi
        if angle < -np.pi:
            angle = angle + 2.0 * np.pi
        return angle

    #线性化
    def linearization(slip_angle,v,steering_angle):
        A=np.zeros((NX,NX))
        for i in range(A.shape[0]):
            A[i,i]=1
        A[0,2]=DT*np.cos(slip_angle)
        A[0,3]=-v*DT*np.sin(slip_angle)
        A[1,2]=DT*np.sin(slip_angle)
        A[1,3]=-v*DT*np.cos(slip_angle)
        A[3,2]=DT*np.tan(steering_angle)/WB

        B=np.zeros((NX,NU))
        B[2,0]=DT
        B[3,1]=v*DT/(WB*np.cos(steering_angle)**2)

        C = np.zeros(NX)
        C[0] = DT * v * np.sin(slip_angle) * slip_angle
        C[1] = - DT * v * np.cos(slip_angle) * slip_angle
        C[3] = - DT * v * steering_angle / (WB * np.cos(steering_angle) ** 2)

        return A,B,C

    def state_update(state,steering_angle,a):
        if steering_angle > MAX_STEER:
            steering_angle = MAX_STEER
        if steering_angle < MIN_STEER:
            steering_angle = MIN_STEER
        state.x = state.x + state.v * np.cos(state.yaw)
        state.y = state.y + state.v * np.sin(state.yaw)
        state.v = state.v + a * DT
        state.yaw = state.yaw + state.v * np.tan(steering_angle)/WB
        if state.v > MAX_SPEED:
            state.v = MAX_SPEED
        elif state.v < MIN_SPEED:
            state.v = MIN_SPEED
        return state

    def nparray(x):
        return np.array(x).flatten()













