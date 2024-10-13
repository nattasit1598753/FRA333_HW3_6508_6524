# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.จิรภัทร_6508
2.ณัฐสิทธิ์_6524
'''
from FRA333_HW3_6508_6524 import *
from spatialmath import SE3
import roboticstoolbox as rtb

#สร้าง MDH-Parameters ด้วย robotic toolbox
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
# q = [pi,pi,pi]
# jacobian_mine = endEffectorJacobianHW3(q)
# print("My jacobian")
# print(jacobian_mine)

# jacobian_toolbox = robot.jacobe(q)
# print("Toolbox jacobian")
# print(jacobian_toolbox)


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

# import random #นำเข้าไลบลารี่ random
# def random_q(range_start, range_end, array_size):#สร้างฟังก์ชั่นเพื่อสุ่มค่า q
#   return [random.uniform(range_start, range_end) for _ in range(array_size)]#ส่งค่า q ที่สุ่มแล้วกลับออกมา

# for i in range(10):#วนเพื่อทดสอบเคสต่าง ๆ 10 ครั้ง
#     q = random_q(0, pi, 3)#สุ่มค่า q ระหว่าง 0 - pi มี array_size 3
#     print("test case q is", q)#แสดงค่า q ที่สุ่มได้
#     print("my function flag is", checkSingularityHW3(q))#แสดง flag ที่ได้จากฟังก์ชั่นของตัวเอง


#     Jacobian_matrix = robot.jacobe(q)#สร้าง jacobian matrix ด้วย robotics toolbox
#     det = abs(np.linalg.det(Jacobian_matrix[:3,:]))#
#     if det < 0.001: # หากค่าที่ได้น้อยกว่า E 
#         print("robotics toolbox flag is",'1') #แสดง 1 กลับคือใกล้ติด Singularity
#     else: #หากค่าที่ได้ไม่น้อยกว่า E 
#         print("robotics toolbox flag is",'0') #แสดง 0 กลับคือไม่ติด Singularity
#     print("end" , i+1 , "round")#แสดงว่าจบ 1 เคส


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
# code here
import random #นำเข้าไลบลารี่ random
def random_q(range_start, range_end, array_size):#สร้างฟังก์ชั่นเพื่อสุ่มค่า q
  return [random.uniform(range_start, range_end) for _ in range(array_size)]#ส่งค่า q ที่สุ่มแล้วกลับออกมา

def random_wrench(array_size):
    wrench = [random.randint(1, 10) for _ in range(array_size - 3)]
    wrench.extend([0, 0, 0])
    return wrench

for i in range(10):
    wrench = random_wrench(6)
    q = random_q(0,pi,3)
    print("my Tua     ", computeEffortHW3(q, wrench))
    Jacobian_matrix = robot.jacobe(q)#สร้าง jacobian matrix ด้วย robotics toolbox
    w = wrench
    J = Jacobian_matrix
    Tua_toolbox = robot.pay(w,q,J)
    print("toolbox Tua", Tua_toolbox)

#==============================================================================================================#
# robot.plot(q,block=True)
