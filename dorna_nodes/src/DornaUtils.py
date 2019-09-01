from sympy import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




def draw_circle():
    thetas = np.linspace(0., 2. * np.pi, 50)
    radius = 0.1
    xs = (radius * np.cos(thetas) + np.ones_like(thetas) * .1).reshape(-1, 1)
    ys = (radius * np.sin(thetas) + np.ones_like(thetas) * .1).reshape(-1, 1)
    zs = (np.ones_like(xs) * 0.3).reshape(-1, 1)
    return np.concatenate((xs, ys, zs), axis=1)  # ARM IN STRAIGHT CONFIG


def get_jacobian(thetas):
    t0, t1, t2, t3 = symbols("t0 t1 t2 t3")

    a = .09548
    d = 0.20603
    l1 = .20327
    l2 = .15245
    l3 = .044

    x = cos(t0)*(a+l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3))
    y = sin(t0)*(a+l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3))
    z = d+l1*sin(t1)+l2*sin(t1+t2)+l3*sin(t1+t2+t3)

    J = Matrix([[diff(x, t0), diff(x, t1), diff(x, t2), diff(x, t3)],
                [diff(y, t0), diff(y, t1), diff(y, t2), diff(y, t3)],
                [diff(z, t0), diff(z, t1), diff(z, t2), diff(z, t3)]])

    return np.array(J.evalf(subs={t0: thetas[0], t1: thetas[1], t2: thetas[2], t3: thetas[3]})).astype(np.float64)

def get_jacobian_arm():
    t0, t1, t2, t3 = symbols("t0 t1 t2 t3")

    a = .09548
    d = 0.20603
    l1 = .20327
    l2 = .15245
    l3 = .044

    # x = cos(t_0) * (a + l1 * cos(t_1) + l2 * cos(t_1 + t_2))
    # y = sin(t_0) * (a + l1 * cos(t_1) + l2 * cos(t_1 + t_2))
    # z = d + l1 * sin(t_1) + l2 * sin(t_1 + t_2)

    x = a+l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3)
    y = a+l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3)
    z = d+l1*sin(t1)+l2*sin(t1+t2)+l3*sin(t1+t2+t3)

    J = Matrix([[diff(x, t1), diff(x, t2), diff(x, t3)],
                [diff(y, t1), diff(y, t2), diff(y, t3)],
                [diff(z, t1), diff(z, t2), diff(z, t3)]])


    det = J
    sing1 = -1.94521
    sing1_ = 1.94521
    sing2 = [1.30103, 2.03241]
    sing2_ = [-1.30103, -2.03241]
    sing3 = [1.84254, 0]
    sing3_ = [-1.84254, 0]
    print(det)


def jacobian_determinant(thetas):
    jac = get_jacobian(thetas)
    det_jac = np.sqrt(np.linalg.det(np.dot(jac.T, jac)))
    return det_jac


def forward_kinematics(thetas):
    alpha = [np.pi/2., 0., 0., 0., 0.]
    a = [.09548, .20327, .15245, .044, 0.]
    d = [0.20603, 0., 0., 0., 0.]
    T = []
    for i in range(len(alpha)):
        T.append(get_transform(thetas[i], alpha[i], a[i], d[i]))

    joint_0 = np.array([[np.cos(thetas[0]), -np.sin(thetas[0]), 0., 0.],
                        [np.sin(thetas[0]), np.cos(thetas[0]), 0., 0.],
                        [ 0., 0., 1., 0.],
                        [0., 0., 0., 1.]])
    joint_1 = T[0]
    joint_2 = np.matmul(joint_1, T[1])
    joint_3 = np.matmul(joint_2, T[2])
    joint_4 = np.matmul(joint_3, T[3])
    joint_5 = np.matmul(joint_4, T[4])
    joints = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5]

    return joints


def get_transform(theta, alpha, a, d):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta)*np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0., np.sin(alpha), np.cos(alpha), d],
        [0., 0., 0., 1.]
    ])


def euler_to_rot(r,p,y):
    cr = np.cos(r)
    sr = np.sin(r)
    cp = np.cos(p)
    sp = np.sin(p)
    cy = np.cos(y)
    sy = np.sin(y)

    return np.array([[cp*cy, -cp*sy, sp],
                     [sr*sp*cy+cr*sy, -sr*sp*sy+cr*cy, -sr*cp],
                     [-cr*sp*cy+sr*sy, cr*sp*sy+sr*cy, cr*cp]])


def rot_to_euler(rot):
    c2 = np.sqrt(rot[0, 0]**2+rot[0, 1]**2)

    roll = np.arctan2(rot[1, 2], rot[2, 2])
    s1 = np.sin(roll)
    c1 = np.cos(roll)

    pitch = np.arctan2(-rot[0, 2], c2)
    yaw = np.arctan2(s1*rot[0, 2]-c1*rot[1, 0], c1*rot[1, 1]-s1*rot[2, 1])

    return roll, pitch, yaw


def inverse_kinematics(pose, p, r):
    #pose = X Y Z R P Y
    # print(pose)
    a = 0.09548
    l1 = 0.20327
    l2 = 0.15245
    l3 = 0.044
    d = 0.20603
    x = pose[0]#, -1]#pose[0]
    y = pose[1]#, -1]
    z = pose[2]#, -1]
    # rot = pose[0:3, 0:3]#euler_to_rot(pose[3], pose[4], pose[5])
    # roll, pitch, yaw = rot_to_euler(rot)

    #SOLVE FOR THETA 0
    theta_0 = np.nan_to_num(np.arctan2(y, x))
    x_ = ((np.cos(theta_0)*x+np.sin(theta_0)*y)-a)
    z_ = z-d
    #SOLVE FOR THETA 2
    c2 = (x_**2+z_**2-l1**2-l2**2)/(2.*l1*l2)
    s2_pos = np.nan_to_num(np.sqrt(1.-c2**2)) #ELBOW DOWN
    s2_neg = -np.nan_to_num(np.sqrt(1.-c2**2)) #ELBOW UP
    theta_2 = np.arctan2(s2_neg, c2)
    #SOLVE FOR THETA 1
    k1 = l1+l2*c2
    k2 = l2*s2_neg
    theta_1 = np.arctan2(z_, x_)-np.arctan2(k2, k1)
    #SOLVE FOR THETA 3
    theta_3 = p-(theta_1+theta_2)
    #SOLVE FOR THETA_4
    theta_4 = r
    thetas = [theta_0, theta_1, theta_2, theta_3, theta_4]
    return thetas

def plot(joints, col, ax):
    for i in range(len(joints)):
        #PLOT JOINT
        ax.scatter(joints[i][0, -1], joints[i][1, -1], joints[i][2, -1], c='k')

        #PLOT LINK
        if i < len(joints)-1:
            ax.plot([joints[i][0, -1], joints[i+1][0, -1]], [joints[i][1, -1], joints[i+1][1, -1]],
                    [joints[i][2, -1], joints[i+1][2, -1]], c = col)

        #PLOT JOINT AXIS
        # u = .1
        # xs = (joints[i][0:3, -1]+joints[i][0:3, 0]*u)
        # ys = (joints[i][0:3, -1]+joints[i][0:3, 1]*u)
        # zs = (joints[i][0:3, -1]+joints[i][0:3, 2]*u)
        # # xs = joints[i][0:3, -1] * (1. - u) + u * norm_orientation[0:3, 0]
        # # ys = joints[i][0:3, -1]*(1.-u)+u*norm_orientation[0:3, 1]
        # # zs = joints[i][0:3, -1]*(1.-u)+u*norm_orientation[0:3, 2]
        # ax.plot([joints[i][0, -1], xs[0]], [joints[i][1, -1], xs[1]], [joints[i][2, -1], xs[2]], c='r')
        # ax.plot([joints[i][0, -1], ys[0]], [joints[i][1, -1], ys[1]], [joints[i][2, -1], ys[2]], c='g')
        # ax.plot([joints[i][0, -1], zs[0]], [joints[i][1, -1], zs[1]], [joints[i][2, -1], zs[2]], c='b')


def init_plot():
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim((-.5, .5))
    ax.set_ylim((-.5, .5))
    ax.set_zlim((-.5, .5))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


def get_constants(initial_theta, final_theta, time_start, time_step):
    time_final = time_start + time_step
    traj_mat = np.array([[1., time_start, time_start ** 2, time_start ** 3],
                         [0., 1., time_start * 2., 3. * time_start ** 2],
                         [1., time_final, time_final ** 2, time_final ** 3],
                         [0., 1., time_final * 2., 3. * time_final ** 2]])
    b = np.array([initial_theta, 0., final_theta, 0.])
    return np.dot(np.linalg.pinv(traj_mat), b.T)


if __name__=="__main__":
    # a = 0.09548
    # l1 = 0.20327
    # l2 = 0.15245
    # l3 = 0.044
    # d = 0.20603
    # ax = init_plot()
    # initial_thetas = [0., 0., 0., 0., 0.]
    # joints = forward_kinematics(initial_thetas)
    # # print(joints[1][0:3,0:3])
    # plot(joints, 'r', ax)
    #
    # initial_thetas = [np.pi/2., np.pi/4., -np.pi/4., 0., 0.]
    # joints = forward_kinematics(initial_thetas)
    # # print(joints[1][0:3,0:3])
    # plot(joints, 'r',ax)
    # # initial_thetas = [-np.pi/2., np.pi/4., -np.pi/4., 0., 0.]
    # # joints = forward_kinematics(initial_thetas)
    # # plot(joints, 'r',ax)
    #
    #
    # # pose = [0., a+np.cos(np.pi/4.)*(l1+l2+l3), d+np.sin(np.pi/4.)*(l1+l2+l3)]
    #
    # theta = inverse_kinematics(joints[-3], 0., 0.)
    # joints_2 = forward_kinematics(theta)
    # plot(joints_2, 'b', ax)
    # plt.show()
    # # print(forward_kinematics(theta))
    get_jacobian_arm()

if __name__ == "__main__":
    print(inverse_kinematics([0.520, 0.041, 0.0927], 0, 0))