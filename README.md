# FRA333_HW3_6508_6524
 
**ข้อที่ 1**

สร้าง function หา jacobian matrix แล้วนำค่าที่ได้ไปเทียบกับค่าจาก robotics toolbox


ตัวอย่างผลลัพธ์เมื่อ q = [pi,pi,pi]
```
My jacobian
[[ 4.94300000e-02  2.66789440e-18 -2.33558512e-17]
 [ 1.39695452e-17 -4.94300000e-02 -4.74430000e-01]
 [-1.09000000e-01 -9.30000000e-02 -9.30000000e-02]
 [ 1.22464685e-16  1.00000000e+00  1.00000000e+00]
 [ 1.00000000e+00 -6.12323426e-17 -6.12323426e-17]
 [ 1.83697015e-16  6.12323559e-17  6.12323559e-17]]
```
```
Toolbox jacobian
[[ 4.94300000e-02  0.00000000e+00  0.00000000e+00]
 [ 2.36705857e-17 -4.94300000e-02 -4.74430000e-01]
 [-1.09000000e-01 -9.30000000e-02 -9.30000000e-02]
 [ 6.12323400e-17  1.00000000e+00  1.00000000e+00]
 [ 1.00000000e+00  0.00000000e+00  0.00000000e+00]
 [ 2.44929360e-16  0.00000000e+00  0.00000000e+00]]
```
 **สรุป** ค่าที่ได้ต่างกันเพียงหลักทศนิยมที่มีค่าเล็ก ๆ จึงสรุปได้ว่าทำถูกต้อง

**ข้อที่ 2**
สุ่มค่า   q ขึ้นมาแล้ว ใช้วิธีเรียกฟังก์ชั่นที่เราสร้างเองเทียบกับการ
เรียกใช้ผ่าน robotics toolbox โดยนำค่า flag มาเปรียบเทียบกันว่าเป็น 0 หรือ 1 ตรงกันหรือไม่

ตัวอย่างผลลัพธ์
```
test case q is [1.8971462340628367, 1.0755906460649824, 0.3359824623362006]
my function flag is      0
robotics toolbox flag is 0

test case q is [1.6687073837614688, 1.7071578516081576, 0.16771436511536042]
my function flag is      0
robotics toolbox flag is 0

test case q is [2.7584588611037546, 2.661917281052559, 2.9323261483372534]
my function flag is      1
robotics toolbox flag is 1
```
**สรุป**
จากการสุ่มเทสเคส q ต่าง ๆ มา จะเห็นได้ว่าฟังชั่นที่เราสร้างเองส่งค่ากลับ 1 หรือ 0 ค่าที่ได้จากการเรียกใช้ผ่าน robotics toolbox ก็จะได้ค่าเดียวกันเสมอ ซึ่งหมายความว่าเราสร้างฟังชั่นถูกต้อง

**ข้อที่ 3**

ตัวอย่างผลลัพธ์
```
my Tua      [4.33223334 3.10868283 3.49758005]
toolbox Tua [4.33223326 3.10868276 3.49758   ]

my Tua      [0.4705816  4.80901775 5.67430007]
toolbox Tua [0.47058156 4.80901766 5.6743    ]

my Tua      [-0.77759006  6.28463221  5.01386991]
toolbox Tua [-0.77759005  6.28463237  5.01387   ]

my Tua      [2.93196289 4.99934195 3.49758008]
toolbox Tua [2.93196294 4.99934179 3.49758   ]

my Tua      [1.39581179 1.82400541 3.78600979]
toolbox Tua [1.39581188 1.82400553 3.78601   ]
```
**สรุป**
ค่าที่ได้จากการเรียกใช้ฟังก์ชั่นที่ตัวเองสร้างแตกต่างจากที่ได้จาก robotics toolbox ในทศนิยมตำแหน่งท้าย ๆ เท่านั้นซึ่งหมายความว่าคำตอบของเราถูกต้อง