## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


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


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Coordinate systems for Modified DH convention: 

![][link_frames]


`O(i)` is the origin for link i frame, and `X(i)`, `Z(i)` are the X and Z axis correspondingly, and Z represents the axis of rotation(translation in case of prismatic joints). Since we are using a right handed coordinate system, `Y(i)` can be calculated accordingly.


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

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


