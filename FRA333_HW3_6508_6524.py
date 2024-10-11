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
    Jacobian_matrix = np.zeros([6, 3])
    for i in range(3):
        p_i = P[:,i]
        Z_i = R[:,2,i]
        linear_velocity = np.cross(Z_i, p_e - p_i)
        Jacobian_matrix[:3, i] = R_e.transpose() @ linear_velocity
        Jacobian_matrix[3:, i] = R_e.transpose() @ Z_i
    return Jacobian_matrix
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    E = 0.001
    Jacobian_matrix = endEffectorJacobianHW3(q)
    det = abs(np.linalg.det(Jacobian_matrix[:3,:]))
    if det < E:
        return 1
    else:
        return 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    Jacobian_matrix = endEffectorJacobianHW3(q)
    Jacobian_matrix_transposed = np.array(Jacobian_matrix).transpose()
    Tao = Jacobian_matrix_transposed @ w
    return Tao
#==============================================================================================================#