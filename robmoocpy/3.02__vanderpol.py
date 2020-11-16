from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def vdp(x1,x2):  
    return x2,-(0.01*(x1**2)-1)*x2-x1
    
    
def f(x,u):
    mx,my,θ,v,δ =list(x[0:5,0])
    u1,u2=list(u[0:2,0])
    return array([[v*cos(δ)*cos(θ)],[v*cos(δ)*sin(θ)],[v*sin(δ)/3],[u1],[u2]])


x    = array([[0],[5],[pi/2],[30],[0.6]])
s=40
dt=0.01
ax=init_figure(-s,s,-s,s)
for t in arange(0,1,dt):
    clear(ax)
    u=array([[0],[0]])
    x=x+dt*f(x,u)
    draw_field(ax,vdp,-s,s,-s,s,4)
    draw_car(x)
pause(1)    



