from roblib import *

def f(x,u):
    x = x.flatten()
    x1,x2,x3,x4 = x[0], x[1], x[2], x[3]
    n3 = (x1**2 + x2**2)**1.5
    v3 = (-x1/n3) + u*x3
    v4 = (-x2/n3) + u*x4
    return array([[x3], [x4], [v3], [v4]])

def control(x):
    x = x.flatten()
    x1,x2,x3,x4 = x[0], x[1], x[2], x[3]
    e1 = x1**2 + x2**2 - R**2
    e2 = x1*x3 + x2*x4
    e3 = x3**2 + x4**2 - 1/R
    k1 = -1
    k2 = -1
    k3 = -1
    u = k1*e1 + k2*e2 + k3*e3
    return u

ech = 2
dt = 0.03
R = 1

ax = init_figure(-ech,ech,-ech,ech)
draw_disk(array([[0],[0]]),R,ax,"grey",0.1,4)
draw_disk(array([[0],[0]]),0.2,ax,"blue",1,1)
x = array([[1.22],[0],[0],[1]]) # position of satellite : x1,x2 / speed vector : x3,x4
draw_disk(array([[x[0]],[x[1]]]),0.1,ax,"red",1)

for t in arange(0,50,dt):
    u = control(x)
    x = x + dt*(0.25*f(x,u) + 0.75*(f(x+dt*(2/3)*f(x,u),u))) # using RK integration method.
    ax.scatter(x[0],x[1],s=0.1,c="black")
    pause(0.001)

waitforbuttonpress()