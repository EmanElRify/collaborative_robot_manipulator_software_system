import numpy as np
import math

## dimensions in meter
a1 = 0.20075
d1 = 0.09
a2 = 0.159
d2 = 0.015

def calculate_forward_kinematics_robot(joint_angles):
 A0_1 = np.array([ [math.cos(joint_angles[0]),-math.sin(joint_angles[0]),0,a1*math.cos(joint_angles[0])],
                   [math.sin(joint_angles[0]),math.cos(joint_angles[0]),0,a1*math.sin(joint_angles[0])],
                   [0,0,1,d1],
                   [0,0,0,1]  ])
 A1_2 = np.array([ [math.cos(joint_angles[1]),-math.sin(joint_angles[1]),0,a2*math.cos(joint_angles[1])],
                   [math.sin(joint_angles[1]),math.cos(joint_angles[1]),0,a2*math.sin(joint_angles[1])],
                   [0,0,1,d2],
                   [0,0,0,1]  ])
 

 A0_2 = np.matmul(A0_1,A1_2)
#  print(A0_2)

 end_effector_pose = np.array([   [A0_2[0,0]],
                               [A0_2[1,0]],
                               [A0_2[2,0]],
                               [A0_2[0,3]],
                                 [A0_2[1,3]],
                                 [A0_2[2,3]]
                                 ])

 return end_effector_pose

# u_x = 0.5
# u_y = math.sin(np.pi/6)
# u_z = 0
#q_x = 0.19
#q_y = 0.30
q_z = d1 + d2

def calculate_inverse_kinematics(end_effector_pose):
    u_x = end_effector_pose[0]
    u_y = end_effector_pose[1]
    u_z = end_effector_pose[2]
    q_x = end_effector_pose[3]
    q_y = end_effector_pose[4]
    q_z = end_effector_pose[5]
    theta_2_cos = (q_x**2 + q_y**2 - a1**2 - a2**2) / (2*a2*a1)
    theta_2 = np.arccos(theta_2_cos)
    #a = a2*np.sin(theta_2)
    #b = a1 + a2 * np.cos(theta_2)
    #c = q_y
    #theta_1_tan_1 = (b + np.sqrt(b**2 + a**2 - c**2)) / (a + c)
    #theta_1 = 2 *np.arctan(theta_1_tan_1)
    #print("theta 1 is {}".format(np.degrees(theta_1)))
    print("theta 2 is {}".format(np.degrees(theta_2)))
    b1=np.arctan(q_y/q_x)
    b2=np.arctan((a2*np.sin(theta_2))/(a1+a2*np.cos(theta_2)))
    theta_1=b1-b2
    if q_x < 0 and q_y > 0:
        theta_1 = theta_1 + np.pi
    #theta_1_tan_2 = (b - np.sqrt(b**2 + a**2 - c**2)) / (a + c)
    #theta_1 = np.arctan(theta_1_tan_2)
    print("theta 1 is {}".format(np.degrees(theta_1)))



    joint_angles = np.array([[theta_1],
                             [theta_2]
                            ])
    joint_angles_degree = np.degrees(joint_angles)
    return joint_angles_degree
    



#test = calculate_forward_kinematics_robot([np.pi/2, np.pi/3])
# print(test)
#angles = calculate_inverse_kinematics([0,0,0,q_x,q_y,q_z])
#print(angles)

