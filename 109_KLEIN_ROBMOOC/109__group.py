from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
from numpy import *


def f(x,u):
    """
    Equations d'etat
    """
    x,u=x.flatten(),u.flatten()
    xdot = array([[x[3]*cos(x[2])],[x[3]*sin(x[2])],[u[0]],[u[1]]])
    return(xdot)

def control(x,w,dw,ddw):
    """
    Loi de commande
    """
    v=w-array([[x[0,0]],[x[1,0]]]) + 2*(dw-array([[x[3,0]*cos(x[2,0])],[x[3,0]*sin(x[2,0])]]))+ddw
    A=array([
        [-x[3,0]*sin(x[2,0])    ,   cos(x[2,0])],
        [x[3,0]*cos(x[2,0])     ,   sin(x[2,0])]])
    u=inv(A)@v
    return u    
    
ax=init_figure(-40,40,-40,40)
m   = 20                # nombre de robots
X   = 10*randn(4,m)     # conditions initiales au hasard
a   = 0.1
dt  = 0.1
for t in arange(0,10,dt):
    clear(ax)
    for i in range(m):
        delta=2*i*pi/m
        theta = a*t

        # Matrice diagonale :
        D=array([
            [20+15*sin(a*t)    ,   0    ],
            [0                 ,   20   ]])
        dD=array([
            [15*a*cos(a*t)    ,   0      ],
            [0                ,  0       ]])
        ddD=array([
            [-15*a**2*sin(a*t) ,    0        ],
            [0                 ,    0       ]])

        # Premiere consigne :
        c   = array([[cos(a*t+delta)],[sin(a*t+delta)]])   
        dc  = a*array([[-sin(a*t+delta)],[cos(a*t+delta)]])   
        ddc = -a**2*c
        # print(ddc.shape)

        # Matrice de rotation :
        R=array([
            [cos(theta),-sin(theta)],
            [sin(theta),cos(theta)]])
        dR=a*array([[-sin(theta),-cos(theta)],[cos(theta),-sin(theta)]])
        ddR=-a**2*R

        # Seconde consigne :
        w=R@D@c
        dw=R@D@dc+R@dD@c+dR@D@c
        ddw=R@D@ddc+R@ddD@c +ddR@D@c +2*dR@D@dc+2*R@dD@dc+2*dR@dD@c

        x   = X[:,i].reshape(4,1)
        # u   = control(x,c,dc,ddc)             # q2
        u   = control(x,w,dw,ddw)               # q3
        x   = X[:,i].reshape(4,1)
        draw_tank(x,'b',0.5)
        x= x+f(x,u)*dt        
        X[:,i]=x.flatten()
        # plot([c[0][0]],[c[1][0]],'r+')        # q2
        plot([w[0][0]],[w[1][0]],'r+')          # q3
show()

"""
q2) Les voitures se repartissent sur le cercle
q3) Les voitures se repartissent sur une ellipsoide et les axes changent (on a toujours un axe qui
vaut 20). Attention,la methode ne gere pas la collision.
"""