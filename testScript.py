# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
'''
from FRA333_HW3_6508_6524 import *
from spatialmath import SE3
import roboticstoolbox as rtb
#==============================================================================================================#
#================================<สร้าง MDH-Parameters ด้วย robotic toolbox>======================================#
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = d_1    ,offset = pi ),
        rtb.RevoluteMDH(alpha = pi/2    ,a = 0.0      ,d = 0.0    ,offset = 0.0),
        rtb.RevoluteMDH(alpha = 0.0     ,a = -a_2     ,d = 0.0    ,offset = 0.0),
    ],
    tool = SE3([
    [0, 0, -1, -(a_3 + d_6)],
    [0, 1, 0, -d_5],
    [1, 0, 0, d_4],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
)


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
# print(endEffectorJacobianHW3([0,0,0]))

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
# print(checkSingularityHW3([0,0,-pi/2]))

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
wrench = [0,-1,0,0,0,0]
q = [0,-pi/2,pi/2]
# q = [0,0,0]
print(computeEffortHW3(q, wrench))


#==============================================================================================================#
robot.plot(q,block=True)
