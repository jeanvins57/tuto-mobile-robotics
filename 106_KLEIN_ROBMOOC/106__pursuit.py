from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    x=x.flatten()
    u=u.flatten()
    return (array([
        [u[0]*cos(x[2])],
        [u[0]*sin(x[2])],
        [u[1]]]))

def control(xa,xb,v):
    print(xa)
    u=array([[5],[1]]) #TO DO
    x0=array([
        [cos(xa[2,0]) ,   sin(xa[2,0])   ,   0],
        [-sin(xa[2,0])  ,   cos(xa[2,0])   ,   0],
        [0          ,       0       ,   1]])
    x=x0@(xb-xa)
    print(x0)
    A=array([
        [-1,x[1,0]],
        [0,-1]])
    b=array([
        [v[0,0]*cos(x[2,0])],
        [v[0,0]*sin(x[2,0])]])
    w=array([[10],[0]]) # toujours la meme distance 10
    dw=array([[0],[0]])
    u=inv(A)@(w-x[0:2]+dw-b)
    return u    

ax=init_figure(-30,30,-30,30)
dt = 0.1
xa = array([[-10], [-10],[0]])
xb = array([[-5],[-5],[0]])
for t in arange(0,10,dt) :
    clear(ax)
    v = array([[3],[sin(0.2*t)]])
    u=control(xa,xb,v)
    draw_tank(xa,'blue')  	
    draw_tank(xb,'red')  	
    xa = xa + dt*f(xa,u)
    xb = xb + dt*f(xb,v)
    