from RobotMove import *
import numpy as np
from sympy import *

q0,q1,q2,q3,q4,q5,q6,q7 = symbols('q0 q1 q2 q3 q4 q5 q6 q7')
def Jacobian(q):
    T = FK(q)
    T[0:3, 3] = 0
    T_inv = np.transpose(T)

    dT = dRz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J1 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*dRx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J2 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*dRz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J3 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*dRx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J4 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*dRz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J5 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*dRx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)*T_inv
    J6 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*dRz(q[6])*Tx(l7)*T_inv
    J7 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    return matrix(np.hstack([J1, J2, J3, J4, J5, J6, J7]), dtype='float')

#print(Jacobian([q0,q1,q2,q3,q4,q5,q6,q7]))