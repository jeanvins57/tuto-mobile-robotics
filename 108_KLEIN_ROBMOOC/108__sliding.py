
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    x=x.flatten()
    u=u.flatten()
    return (array([
        [x[3]*cos(x[2])],
        [x[3]*sin(x[2])],
        [u[0]],
        [u[1]]]))

def control(x,w,dw,ddw):
    A=array([
        [-x[3,0]*sin(x[2,0])    ,       cos(x[2,0])],
        [   x[3,0]*cos(x[2,0])      ,   sin(x[2,0])]])
    y=array([
        [x[0,0]],
        [x[2,0]],])
    dy=array([
        [x[3,0]*cos(x[2,0])],
        [x[3,0]*sin(x[2,0])]]),
    ddy=(w-y)+2*(dw-dy)+ddw
    u=inv(A)@ddy
    return u

ax=init_figure(-30,30,-30,30)
dt = 0.02
x = array([[10],[0],[1],[1]]) # x1,x2,x3(heading),x4(speed)
u = array([[1],[1]])
L=10
s = arange(0,50,0.01)
E=[] # erreur
time=3
for t in arange(0,time,dt) :
    clear(ax)
    plot(L*cos(s), L*sin(3*s),color='magenta')
    draw_tank(x,'red')  
    w=L*array([[cos(t)],[sin(3*t)]])
    dw=L*array([[-sin(t)],[3*cos(t)]]) 
    ddw=L*array([[-cos(t)],[-9*sin(t)]])
    u=control(x,w,dw,ddw)
    E.append(abs(x[0,0]-w[0,0])+abs(x[1,0]-w[1,0]))
    draw_disk(w,0.5,ax,"red")    
    x = x + dt*f(x,u)
    print("time=",t)
    print('error=',E)

figure()
x=arange(0,time,dt)
plot(E)
show()