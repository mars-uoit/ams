#!/usr/bin/env python

import math

# Link length
L_12 = 0.06985
L_23 = 0.14605
L_34 = 0.18500
L_4G = 0.08730

# Desired position and gripper angle relative to x
X = 0.20
Y = 0
Z = 0.20
alpha = 0
print 'Desired position:', X, Y, Z, 'and desired gripper angle:', alpha
# IK solution

# Joint 1
j1 = math.atan2(Y,X)

# Joint 2
Xp = math.cos(j1)*X + math.sin(j1)*Y
Zp = Z - L_12
a2 = Xp - math.cos(alpha)*L_4G
b2 = Zp - math.sin(alpha)*L_4G
k2 = (math.pow(a2,2) + math.pow(b2,2) + math.pow(L_23,2) - math.pow(L_34,2))/(2*L_23)
q2 = math.pow(a2,2) + math.pow(b2,2) - math.pow(k2,2)
j2 = math.atan2(b2,a2) - math.atan2(math.sqrt(round(q2,4)),k2)
j2p = math.atan2(b2,a2) + math.atan2(math.sqrt(round(q2,4)),k2)
print j2, j2p
if j2 <= 0:
  j2 = j2p
# Joint 3
#a3 = (Xp - math.cos(alpha)*L_4G - L_23)/(L_34)
#b3 = (Zp - math.sin(alpha)*L_4G - L_23)/(L_34)
#c3 = a3*math.cos(j2) + b3*math.sin(j2)
#s3 = b3*math.cos(j2) + a3*math.sin(j2)
#j3 = math.atan2(s3,c3)
a3 = (a2 - math.cos(j2)*L_23)/(L_34)
b3 = (b2 - math.sin(j2)*L_23)/(L_34)
j23 = math.atan2(b3,a3)
j3 = j23 - j2

# Joint 4
j4 = alpha - j2 - j3
# Display joints for IK
print 'The IK solution is :', j1, j2, j3, j4

# FK solution
Xc = L_23*math.cos(j1)*math.cos(j2) + L_34*math.cos(j1)*math.cos(j2+j3) + L_4G*math.cos(j1)*math.cos(j2+j3+j4)
Yc = L_23*math.sin(j1)*math.cos(j2) + L_34*math.sin(j1)*math.cos(j2+j3) + L_4G*math.sin(j1)*math.cos(j2+j3+j4)
Zc = L_12 + L_23*math.sin(j2) + L_34*math.sin(j2+j3) + L_4G*math.sin(j2+j3+j4)
print 'Using the solution form the IK and subing into FK to get:', Xc, Yc, Zc
app = j2 + j3 + j4
print app
