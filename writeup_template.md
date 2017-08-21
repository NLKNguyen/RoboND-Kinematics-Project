Project: Kinematics Pick & Place
================================



[//]: # (Image References)

[link_frames]: ./misc_images/link_frames.png
[dh-transform]: ./misc_images/dh-transform-matrix.png
[T0_1]: ./misc_images/T0_1.gif
[T1_2]: ./misc_images/T1_2.gif
[T2_3]: ./misc_images/T2_3.gif
[T3_4]: ./misc_images/T3_4.gif
[T4_5]: ./misc_images/T4_5.gif
[T5_6]: ./misc_images/T5_6.gif
[T6_EE]: ./misc_images/T6_EE.gif
[theta1]: ./misc_images/theta1.gif
[theta2]: ./misc_images/theta2.gif
[theta3]: ./misc_images/theta3.gif
[theta4]: ./misc_images/theta4.gif
[theta5]: ./misc_images/theta5.gif
[theta6]: ./misc_images/theta6.gif
[sides]: ./misc_images/sides.png
[angles]: ./misc_images/angles.png




# Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Coordinate systems for Modified DH convention: 

![][link_frames]

**Modified DH Parameter** table

Links | alpha(i-1) | a(i-1)  | d(i-1) | theta(i)
---   | ---        | ---     | ---    | ---
0->1  | 0          | 0       | 0.75   | qi
1->2  | - pi/2     | 0.35    | 0      | -pi/2 + q2
2->3  | 0          | 1.25    | 0      | q3
3->4  | - pi/2     | - 0.054 | 1.5    | q4
4->5  | pi/2       | 0       | 0      | q5
5->6  | - pi/2     | 0       | 0      | q6
6->EE | 0          | 0       | 0.303  | 0

`alpha(i-1)` : represents the angle between Z(i-1) and Z(i) along X(i-1)   

`a(i-1)` : Link length, represents the distance between Z(i-1) and Z(i) along X(i-1)   
`d(i)`  : Link offset, represents the distance between X(i-1) and X(i) along Z(i-1)

`q(i)` : Joint angle,represents the angle between X(i-1) and X(i) along Z(i)

`Link 0->1` represents the base link   

`Link 6->EE` represents the gripper link

These are represented in Python code as a dictionary:

```python
DH_Table = {
    alpha0: 0,        a0: 0,      d1: 0.75,  q1: q1,
    alpha1: -pi/2.,   a1: 0.35,   d2: 0,     q2: -pi/2. + q2,
    alpha2: 0,        a2: 1.25,   d3: 0,     q3: q3,
    alpha3: -pi / 2., a3: -0.054, d4: 1.5,   q4: q4,
    alpha4: pi / 2.,  a4: 0,      d5: 0,     q5: q5,
    alpha5: -pi / 2., a5: 0,      d6: 0,     q6: q6,
    alpha6: 0,        a6: 0,      d7: 0.303, q7: 0,
}
``` 

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

General form of matrix transformation  between two frames
![DH Transform][dh-transform] 
  
Function implementation for Kuka KR210 matrix transformation:
``` python
# Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle

# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')


# Define modified DH transformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([
            [cos(q),            -sin(q),           0,           a             ] ,
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d ] ,
            [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d  ] ,
            [0,                 0,                 0,           1             ]
        ])
    return TF

```

This function is called many times to construct the transformation matrices between the joint frames using the corresponding DH parameters stored in `DH_Table` above.
```Python
# Create individual transformation matrices
T0_1  = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
```
![T0_1]


```Python
T1_2  = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
```
![T1_2]

```Python
T2_3  = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
```
![T2_3]

```Python
T3_4  = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
```
![T3_4]

```Python
T4_5  = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
```
![T4_5]

```Python
T5_6  = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
```
![T5_6]

```Python
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```
![T6_EE]


Then we multiply these matrices to obtain the pose of the end-effectors' frame expressed in the base frame:

```Python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

Obtain the final homogenous transformation in euler angles from quaternion representation of the end-effector orientation
```Python
# Extract end-effector position and orientation from request
# px,py,pz = end-effector position
# roll, pitch, yaw = end-effector orientation
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w])

```

```Python
r, p, y = symbols('r p y')

ROT_x = Matrix([ [1,      0, 0       ],
                 [0, cos(r), -sin(r) ],
                 [0, sin(r), cos(r)  ] ]) # ROLL 

ROT_y = Matrix([ [cos(p),  0, sin(p) ],
                 [0,       1, 0      ],
                 [-sin(p), 0, cos(p) ] ]) # PITCH

ROT_z = Matrix([ [cos(y), -sin(y), 0 ],
                 [sin(y),  cos(y), 0 ],
                 [0,            0, 1 ] ]) # YAW

ROT_EE = ROT_z * ROT_y * ROT_x

ROT_ERROR = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

ROT_EE = ROT_EE * ROT_ERROR

ROT_EE_evaled = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


Compute wrist center position using the end-effector pose:
```Python
EE = Matrix([ [px],
              [py],
              [pz] ])


WC = EE - (0.303) * ROT_EE_evaled[:,2]
```

**Inverse Position Kinematics**

**Theta1** computation: ![theta1]

```Python
theta1 = atan2(WC[1], WC[0])
```

**Theta2** and **theta3** computation: 

SSS triangle for theta2 and theta3

![sides]

![angles]




```Python
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + 
                pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / 
                (2 * side_b * side_c))

angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / 
                (2 * side_a * side_c))

angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / 
                (2 * side_a * side_b))

```

![theta2]
```Python
theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
```

![theta3]
```Python
theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m
```

**Inverse Orientation Kinematics**

**Theta4**, **theta5**, and **theta6** computation:
```Python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

R3_6 = R0_3.inv("LU") * ROT_EE_evaled
```

![theta4]
```Python
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
```

![theta5]
```Python
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
```

![theta6]
```Python
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
# Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The implementation is filled in 2 parts as the provided `IK_server.py` suggests. The forward kinematics part is explained in section 1 and 2 in the above analysis, and the inverse kinematics part is explained in section 3.

Possible improvements:

* Make it run faster by caching parts that are recomputed many times.

* Instead of going with the only IKM solution that it finds, generate multiple solutions and select the best one can reduce parts of further computations along the way. Moreover, that also reduces electric consumption when running on a real robot.



