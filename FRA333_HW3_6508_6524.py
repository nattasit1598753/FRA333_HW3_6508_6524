# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.
2.
3.
'''

from HW3_utils import FKHW3
import math
import numpy as np
pi = math.pi
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R,P,R_e,p_e = FKHW3(q)
    Jacobian_matrix = np.zeros([6, 3]) #สร้างเมทริก 6*3 โดยทุกตัวมีค่า 0
    for i in range(3):#สั่งวนลูป 3 ครั้งเพราะมี 3 joint
        p_i = P[:,i]#ดึงค่า P ทุกแถวในคอลั่มน์ i
        Z_i = R[:,2,i]#ดึงค่า R ทุกแถวในคอลั่มน์ 3 ของชุดที่ i
        linear_jacobian = np.cross(Z_i, p_e - p_i)#ใช้สูตร Zi cross (Pe - Pi) ให้ได้ linear jacobian velocity
        Jacobian_matrix[:3, i] = R_e.transpose() @ linear_jacobian #ใส่ linear_jacobian เข้าไปใน 3 แถวบนของjacobian matrix และแปลงเฟรมให้เข้าเฟรมe
        Jacobian_matrix[3:, i] = R_e.transpose() @ Z_i#ใส่ Z_i เข้าไปใน 3 แถวล่างของjacobian matrix และแปลงเฟรมให้เข้าเฟรมe
    return Jacobian_matrix #ส่งค่า Jacobian_matrix กลับ
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    E = 0.001 #กำหนดค่า E ตามโจทย์
    Jacobian_matrix = endEffectorJacobianHW3(q) #เรียกใช้ฟังก์ชั่นหา jacobian ที่เราสร้างไว้
    det = abs(np.linalg.det(Jacobian_matrix[:3,:])) #หาขนาดของ det ของ jacobian matrix ที่ลดรูปเหลือแค่ linear jacobian
    if det < E: # หากค่าที่ได้น้อยกว่า E 
        return 1 #ส่งค่า 1 กลับคือใกล้ติด Singularity
    else: #หากค่าที่ได้ไม่น้อยกว่า E 
        return 0 #ส่งค่า 0 กลับคือไม่ติด Singularity
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    Jacobian_matrix = endEffectorJacobianHW3(q)
    Jacobian_matrix_transposed = np.array(Jacobian_matrix).transpose()
    Tua = Jacobian_matrix_transposed @ w
    return -Tua
#==============================================================================================================#