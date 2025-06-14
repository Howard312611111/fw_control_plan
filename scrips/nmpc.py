import rospy
import sys
import math
import numpy as np
import cv2
from math import atan2
from math import cos
from math import sin
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from fw_control.msg import EstimateOutput
from fw_control.msg import Trajectory3D
from tf.transformations import euler_from_quaternion
from casadi import *
from message_filters import ApproximateTimeSynchronizer, Subscriber

# The time of the entire predict horizon is recommended to be greater than 1 second.
# In other words, N >= loop_rate
loop_rate = 40
T = 1./loop_rate        # Sampling Time
N = 50                  # Predict horizon
deg2rad = pi/180
gravity = 9.81

sd_ = DM([(cx - cx)/fx, (cy - cy)/fy, 50])

host_gps = PoseStamped()
host_gps_vel = TwistStamped()

def Orientation2Euler(msg):
    orientation_list = [msg.x, msg.y, msg.z, msg.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return roll, pitch, yaw

def BuildRotationMatrix(axis, ang):
    rotation = np.mat(np.zeros((3,3)), dtype = float)
    if(ang == 'X'):
        rotation = np.mat([[1,         0,         0],
                             [0,  cos(ang), sin(ang)],
                             [0, -sin(ang), cos(ang)]])
    
    if(ang == 'Y'):
        rotation = np.mat([[cos(ang), 0, -sin(ang)],
                             [       0, 1,         0],
                             [sin(ang), 0,  cos(ang)]])
    
    if(ang == 'Z'):
        rotation = np.mat([[ cos(ang), sin(ang), 0],
                             [-sin(ang), cos(ang), 0],
                             [        0,        0, 1]])
    return rotation

def callback(msg1, msg2, msg5):
    print('here i am')
    
    global tar_vel, tar_pos, x_state    # estimated value
    tar_vel = np.array([0, 0, 0]) #np.array([msg3.target_vel.x, msg3.target_vel.y, msg3.target_vel.z])
    tar_pos = np.array([0, 0, 0]) #np.array([msg3.target_pose.x, msg3.target_pose.y, msg3.target_pose.z])
    x_state = np.array([0, 0, 0]) #np.array([msg3.feature_1.data, msg3.feature_2.data, msg3.feature_3.data])

    global pt_ang, pt_vel               # gimbal info
    pt_ang = np.array([msg2.position[0], msg2.position[1], msg2.position[2]])
    pt_vel = np.array([msg2.velocity[0], msg2.velocity[1], msg2.velocity[2]])

    global uav_pos, uav_vel, uav_wvel   # uav info
    host_gps.pose = msg1.pose.pose
    host_gps_vel.twist = msg1.twist.twist
    uav_pos = np.array([host_gps.pose.position.x,host_gps.pose.position.y,host_gps.pose.position.z])
    uav_vel = np.array([host_gps_vel.twist.linear.x,host_gps_vel.twist.linear.y,host_gps_vel.twist.linear.z])
    uav_wvel = np.array([host_gps_vel.twist.angular.x,host_gps_vel.twist.angular.y,host_gps_vel.twist.angular.z])

    global tar_acc                      # estimated value
    tar_acc = np.array([0, 0, 0]) #np.array([msg4.acc.x, msg4.acc.y, msg4.acc.z])

    global cam_info                     # camera info
    cam_info = np.array([msg5.width, msg5.height, msg5.K[2], msg5.K[5], msg5.K[0], msg5.K[4]])  # [w,h,cu,cv,fx,fy]
    


# Declare model variables
x1 = SX.sym('x1')       # C-G: x1 in {C}
x2 = SX.sym('x2')       # C-G: x2 in {C}  
x3 = SX.sym('x3')       # C-G: x3 in {C} 
x4 = SX.sym('x4')       # G: pan angle              
x5 = SX.sym('x5')       # G: tilt angle
x6 = SX.sym('x6')       # F: position of x in {E}
x7 = SX.sym('x7')       # F: position of y in {E}
x8 = SX.sym('x8')       # F: position of z in {E}
x9 = SX.sym('x9')       # F: roll angle
x10 = SX.sym('x10')     # F: pitch angle
x11 = SX.sym('x11')     # F: yaw angle
x = vertcat(x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11)

u1 = SX.sym('u1')       # G: pan angle rate     (w_c3)
u2 = SX.sym('u2')       # G: tilt angle rate    (w_c2)
u3 = SX.sym('u3')       # F: airspeed           (Vf)
u4 = SX.sym('u4')       # F: roll rate          (w_1)
u5 = SX.sym('u5')       # F: pitch rate         (w_2)
u = vertcat(u1, u2, u3, u4, u5)

u1p = SX.sym('u1p')       # G: pan angle rate     (w_c3)
u2p = SX.sym('u2p')       # G: tilt angle rate    (w_c2)
u3p = SX.sym('u3p')       # F: airspeed           (Vf)
u4p = SX.sym('u4p')       # F: roll rate          (w_1)
u5p = SX.sym('u5p')       # F: pitch rate         (w_2)
upr = vertcat(u1p, u2p, u3p, u4p, u5p)

vq1 = SX.sym('vq1')     # T: velocity of x in {E}
vq2 = SX.sym('vq2')     # T: velocity of y in {E}
vq3 = SX.sym('vq3')     # T: velocity of z in {E}
vq = vertcat(vq1, vq2, vq3)

pm1 = SX.sym('pm1')     # F-G: position of x in {C}
pm2 = SX.sym('pm2')     # F-G: position of y in {C}
pm3 = SX.sym('pm3')     # F-G: position of z in {C}
pm = vertcat(pm1, pm2, pm3)

sd1 = SX.sym('sd1')
sd2 = SX.sym('sd2')
sd3 = SX.sym('sd3')
sd = vertcat(sd1, sd2, sd3)

# rotation matrices
R_g2l = SX.sym('R_g2l', 3, 3)
R_l2b = SX.sym('R_l2b', 3, 3)
R_b2p = SX.sym('R_b2p', 3, 3)
R_p2c = SX.sym('R_p2c', 3, 3)

R_p2t = SX.sym('R_p2t', 3, 3)
R_t2c = SX.sym('R_t2c', 3, 3)

# uav velocity in {C}
R_g2c = SX.sym('R_g2c', 3, 3)
#FVelG2c = Function('FVelG2c', [x10, x11, u3, R_g2c], [mtimes(R_g2c, vertcat( u3*cos(x11)*cos(x10), u3*sin(x11)*cos(x10), u3*sin(x10) ))])
vp_ = mtimes(R_g2c, vertcat( u3*cos(x11)*cos(x10), u3*sin(x11)*cos(x10), u3*sin(x10) )) #FVelG2c(x10, x11, u3, R_g2c)

# uav angular velocity in {C}
R_l2c = SX.sym('R_l2c', 3, 3)
#FWvelL2c = Function('FWvelL2c', [x9, x10, u3, u4, u5, R_l2c], [mtimes(R_l2c, vertcat( u4, u5, -gravity/u3*atan(x9)*cos(x10) ))])
wp_ = mtimes(R_l2c, vertcat( u4, u5, -gravity/u3*atan(x9)*cos(x10) )) #FWvelL2c(x9, x10, u3, u4, u5, R_l2c)

# uav angular velocity in {C}
#GWvelP2c = Function('GWvelP2c', [u1, u2, R_p2c, R_t2c], [mtimes(R_p2c, vertcat(0, 0, u1))+mtimes(R_t2c, vertcat(0, u2, 0))])
wc_ = mtimes(R_p2c, vertcat(0, 0, u1))+mtimes(R_t2c, vertcat(0, u2, 0)) #GWvelP2c(u1, u2, R_p2c, R_t2c)


# Dynamics varables
zeta1 = - wc_[1]*(1+x1**2) + wc_[2]*x2
zeta2 = - wc_[1]*x1*x2 - wc_[2]*x1
zeta3 = - wc_[1]*x1*x3
omega1 = wp_[0]*x1*(x2+pm2*x3) - wp_[1]*(1+x1**2+x3*(pm3+pm1*x1)) + wp_[2] *(x2+pm2*x3)
omega2 = wp_[0]*(1+x2**2+x3*(pm3+pm2*x2)) - wp_[1]*x2*(x1+pm1*x3) - wp_[2] *(x1+pm1*x3)
omega3 = wp_[0]*x3*(x3*pm2+x2) - wp_[1]*x3*(x3*pm1+x1)
eta1 = ( vp_[2]*x1 - vp_[0] )*x3
eta2 = ( vp_[2]*x2 - vp_[1] )*x3
eta3 = vp_[2]*x3**2

# Predict Model equations
x1dot = vq1*x3 - vq3*x1*x3 + zeta1 + omega1 + eta1
x2dot = vq2*x3 - vq3*x2*x3 + zeta2 + omega2 + eta2
x3dot = -vq3*x3**2 + zeta3 + omega3 + eta3
x4dot = u1
x5dot = u2
x6dot = u3*cos(x11)*cos(x10)
x7dot = u3*sin(x11)*cos(x10)
x8dot = u3*sin(x10)
x9dot = u4
x10dot = u5
x11dot = -gravity/u3*atan(x9)*cos(x10)
xdot = vertcat(x1dot, x2dot, x3dot, x4dot, x5dot, x6dot, x7dot, x8dot, x9dot, x10dot, x11dot)

# Multiple Function
A = SX.sym('A',3,3)
B = SX.sym('B',3)
mul = Function('mul',[A,B],[mtimes(A,B)])

# Objective term
Qs = DM.eye(3)
Rs = DM.eye(5)
Ws = DM.eye(1)
Qs[0,0] = 1
Qs[1,1] = 1
Qs[2,2] = 1
Rs[0,0] = 1
Rs[1,1] = 1
Rs[2,2] = 1
Rs[3,3] = 1
Rs[4,4] = 1
Ws[0,0] = 1
L = mtimes( mtimes( ( vertcat(x[0], x[1], x[7]) - sd).T, Qs), ( vertcat(x[0], x[1], x[7]) - sd) ) + mtimes( mtimes( (u-upr).T, Rs), (u-upr) )
# Formulate discrete time dynamics
f = Function('f', [x, u, upr, vq, pm, sd, R_g2c, R_l2c, R_p2c, R_t2c], [xdot, L])
X0 = SX.sym('X0', 11)
U = SX.sym('U', 5)
Upr = SX.sym('Upr', 5)
Vq = SX.sym('Vq', 3)
Pm = SX.sym('Pm', 3)
Sd = SX.sym('Sf', 3)
R_G2C = SX.sym('R_G2L', 3, 3)
R_L2C = SX.sym('R_L2B', 3, 3)
R_P2C = SX.sym('R_P2C', 3, 3)
R_T2C = SX.sym('R_T2C', 3, 3)

X = X0
Q = 0
k1, k1_q = f(X, U, Upr, Vq, Pm, Sd, R_G2C, R_L2C, R_P2C, R_T2C)
X = X + k1*T
Q = Q + k1_q
F = Function('F', [X0, U, Upr, Vq, Pm, Sd, R_G2C, R_L2C, R_P2C, R_T2C], [X, Q],['x0','u','upr','tv','pm','xd','R1','R2','R3','R4'],['xf','qf'])


rospy.init_node('nmpc', anonymous=True)
tss = ApproximateTimeSynchronizer([Subscriber("uav0/base_pose_ground_truth", Odometry), Subscriber("uav0/gimbal/joint_states", JointState), 
                       #Subscriber("uav0/estimation/ukf/output_data", EstimateOutput), Subscriber("/target_qp", Trajectory3D),
                       Subscriber("uav0/camera_ir/camera/color/camera_info", CameraInfo)],1, 0.1, allow_headerless=True)
tss.registerCallback(callback)
rate = rospy.Rate(loop_rate)


# Constant geometry between mount and uav
geo1x = 1.156 #rospy.get_param('miniyy/relPos_panPlane/x')
geo1y = 0.000 #rospy.get_param('miniyy/relPos_panPlane/y')
geo1z = 0.062 #rospy.get_param('miniyy/relPos_panPlane/z')
geo2x = 0.016 #rospy.get_param('miniyy/relPos_camPan/x')
geo2y = 0.000 #rospy.get_param('miniyy/relPos_camPan/y')
geo2z = 0.062 #rospy.get_param('miniyy/relPos_camPan/z')
geo1 = vertcat(geo1x, geo1y, geo1z)
geo2 = vertcat(geo2x, geo2y, geo2z)

for i in range(0,100):
    print('spin')
    rate.sleep()

# Test the initial point
Fk = F(x0=[0.16,0.12,0.05,0.03,0.01,20,30,25,0.01,0.01,0.01], u=[0,0,20,0,0], upr=[0,0,19,0,0], tv=[1,1,0], pm=[0.001,0.001,0.001], xd=sd_, 
       R1=DM([[1,0,0],[0,1,0],[0,0,1]]), R2=DM([[1,0,0],[0,1,0],[0,0,1]]), R3=DM([[1,0,0],[0,1,0],[0,0,1]]), R4=DM([[1,0,0],[0,1,0],[0,0,1]]) )
print(Fk['xf'])
print(Fk['qf'])


while not rospy.is_shutdown():
    # Start with an empty NLP
    w0=[]
    w=[]
    lbw = []
    ubw = []
    g=[]
    lbg = []
    ubg = []
    J = 0

    # Calculate the rotation matrix between {C} and {E}
    rotL2B = np.mat([[1, 0, 0],
                     [0,-1, 0],
                     [0, 0,-1]])
    uav_roll, uav_pitch, uav_yaw = Orientation2Euler(host_gps.pose.orientation)

    
    rotG2L = BuildRotationMatrix('X', uav_roll)*BuildRotationMatrix('Y', uav_pitch)*BuildRotationMatrix('Z', uav_yaw)
    rotB2P = BuildRotationMatrix('Z', pt_ang[2])
    rotP2C = BuildRotationMatrix('X', 1.57)*BuildRotationMatrix('Z', 1.57)*BuildRotationMatrix('Y', pt_ang[1])

    rot_g2b = rotL2B*rotG2L
    rot_b2c = rotP2C*rotB2P
    rot_g2c = rot_b2c*rotL2B*rotG2L
    rot_b2g = rot_g2b.T
    rot_c2b = rot_b2c.T
    rot_c2g = rot_g2c.T

    # Calculate the target position, velocity, acc in {C}
    tar_pos_ = mul(rot_g2c, tar_pos)
    tar_vel_ = mul(rot_g2c, tar_vel)
    tar_acc_ = mul(rot_g2c, tar_acc)

    geo = rotP2C*( (rotB2P*rotL2B*geo1) + geo2 )

    # Calculate the state
    X_state = DM([x_state[0], x_state[1], x_state[2], pt_ang[2], pt_ang[1], uav_pos[0], uav_pos[1], uav_pos[2], uav_roll, uav_pitch, uav_yaw])

    # Formulate the NLP
    # initial condition
    Xk = SX.sym('X0', 11)
    w += [Xk]
    lbw += list(X_state.full().flatten())
    ubw += list(X_state.full().flatten())
    w0 += list(X_state.full().flatten())

    print(w)
    print(w0)
    

    for k in range(N):

        



        
        Uk = SX.sym('U_' + str(k), u.shape[0])
        w += [Uk]
        lbw += [-60*deg2rad, -60*deg2rad, 15, -60*deg2rad, -60*deg2rad]
        ubw += [60*deg2rad, 60*deg2rad, 30, 60*deg2rad, 60*deg2rad]
        w0 += [0, 0, 0, 0, 0]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, xd=sd_, u=Uk, upr=DM(vercat(,)), tv=DM(tar_vel_), pm=DM(geo), R=)
        Xk_end = Fk['xf']
        J = J + Fk['qf']

        # New NLP variable for state at end of interval
        Xk = SX.sym('X_' + str(k+1), 11)
        w   += [Xk]
        lbw += [(0 - cam_info[2])/cam_info[4], (0 - cam_info[3])/cam_info[5], -120*deg2rad, -80*deg2rad, -5000, -5000, 10, -60*deg2rad, -60*deg2rad, -60*deg2rad]
        ubw += [(cam_info[0] - cam_info[2])/cam_info[4], (cam_info[1] - cam_info[3])/cam_info[5], 120*deg2rad, 80*deg2rad, 5000, 5000, 80, 60*deg2rad, 60*deg2rad, 60*deg2rad]
        w0  += [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # update 
        tar_vel_ = tar_vel_ + tar_acc_*T
    
        # Add equality constraint
        g   += [Xk_end - sd_]
        lbg += [0, 0, 0]
        ubg += [0, 0, 0]

    # Create an NLP solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    opts={}
    
    opts["verbose_init"] = False
    opts["verbose"] = False
    opts["print_time"] = False
    opts["ipopt.print_level"] = 0
    
    solver = nlpsol('solver', 'ipopt', prob, opts);

    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

    w_opt = sol['x']
