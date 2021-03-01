import numpy as np
from numpy.linalg import inv, pinv
import matplotlib.pyplot as plt
from RobotMove import *
from Jacobian import Jacobian


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.xlim(-1300, 1300)
plt.ylim(-1300, 1300)

def plotRobot(q):
    pos = [[0, 0, 0],[0, 0, l1]]

    T = Rz(q[0])*Tz(l1)*Rx(q[1])*Tz(l2)
    pos.append(transpose(T[0:3, 3])[0])

    T = T*Rz(q[2])*Tz(l3)
    pos.append(transpose(T[0:3, 3])[0])

    T = T*Rx(q[3])*Tz(l4)
    pos.append(transpose(T[0:3, 3])[0])

    T = T*Rz(q[4])*Tz(l5)
    pos.append(transpose(T[0:3, 3])[0])

    T = T*Rx(q[5])*Tz(l6)
    pos.append(transpose(T[0:3, 3])[0])

    T = T*Rz(q[6])*Tz(l7)
    pos.append(transpose(T[0:3, 3])[0])

    x = [i[0] for i in pos]
    y = [i[1] for i in pos]
    z = [i[2] for i in pos]

    ax.plot(x, y, z)
    ax.scatter(x, y, z)

#def trajectory(pos): ax.scatter(pos[0],pos[1],pos[2])

goal = np.array([1,3,0,0,0,0])
q = np.array([0,0,0,0,0,0,0])

plotRobot(q)

def pseudoInverse(J): return pinv(J)
def WPInv(J): return W.dot(transpose(J))*pinv(matrix(J.dot(W).dot(transpose(J)),dtype='float'))
def LeastSquares(J): return matrix(transpose(J)).dot(pinv(matrix(J.dot(transpose(J))+l*eye(6),dtype='float')))


def nullSpace(q, goal):
    q_dot_0 = [4, 9, 1, 7, 9, 5, 1]
    for i in range(100):
        j_hash = WPInv(Jacobian(q))
        ductTape = Matrix(np.dot(j_hash,(goal-np.hstack([transpose(FK(q)[0:3,3])[0],[0, 0, 0]])))+(np.eye(7)-np.dot((j_hash*Jacobian(q)),q_dot_0)))/100
        q = q + [ductTape[j] for j in range(7)]
        print(i)
    return q


def IK(q,goal, invFunc):
    for i in range(0,800):
        ductTape = Matrix(np.dot(invFunc(Jacobian(q)),(goal - np.hstack ([transpose(FK(q)[0:3, 3])[0], [0, 0, 0]]))/100))
        q = q+[ductTape[j] for j in range(7)]
    return q
def nullIK(q,goal):
    q0 = q
    for i in range(0,800):
        J = Jacobian(q)
        ductTape = Matrix(np.dot(pinv(J),(goal - np.hstack ([transpose(FK(q)[0:3, 3])[0], [0, 0, 0]]))/100-np.reshape(Matrix(J.dot(q0)),(-1,1))))
        q = q+[ductTape[j] for j in range(7)]
    return q
l = 15
W = inv(np.diag([1,2,2,2,2,2,2]))
#q = IK(q,goal,WPInv)
q = nullSpace(q,goal)
plotRobot(q)
plt.show()