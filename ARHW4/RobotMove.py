from numpy import matrix, cos, sin, arctan2, sqrt
from sympy import *

l1,l2,l3,l4,l5,l6,l7 = 340,200,200,200,200,126,4

def cross(a, b): return transpose(matrix([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]))
def Tz(a): return matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, a], [0, 0, 0, 1]])
def Ty(a): return matrix([[1, 0, 0, 0], [0, 1, 0, a], [0, 0, 1, 0], [0, 0, 0, 1]])
def Tx(a): return matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
def dTz(): return matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
def dTy(): return matrix([[0, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]])
def dTx(): return matrix([[0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
def Rx(a): return matrix([[1, 0, 0, 0], [0, cos(a), -sin(a), 0], [0, sin(a), cos(a), 0], [0, 0, 0, 1]])
def Ry(a): return matrix([[cos(a), 0, sin(a), 0], [0, 1, 0, 0], [-sin(a), 0, cos(a), 0], [0, 0, 0, 1]])
def Rz(a): return matrix([[cos(a), -sin(a), 0, 0], [sin(a), cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
def dRx(a): return matrix([[0,0,0,0],[0,-sin(a),-cos(a),0],[0,cos(a),-sin(a),0],[0,0,0,0]])
def dRy(a): return matrix([[-sin(a),0,cos(a),0],[0,0,0,0],[-cos(a),0,-sin(a),0],[0,0,0,0]])
def dRz(a): return matrix([[-sin(a),-cos(a),0,0],[cos(a),-sin(a),0,0],[0,0,0,0],[0,0,0,0]])
def FK(q): return Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)*Rz(q[2])*Tz(l3)*Rx(q[3])*Tz(l4)*Rz(q[4])*Tz(l5)*Rx(q[5])*Tx(l6)*Rz(q[6])*Tx(l7)