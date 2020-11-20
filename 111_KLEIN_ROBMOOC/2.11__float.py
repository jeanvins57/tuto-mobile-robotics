from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def draw_buoy(x):
    clear(ax) 
    x=x.flatten()
    plot([-10,10],[0,0],'black',linewidth=1)    
    d=x[0]
    P=array([[-ech,-1.8*ech],[ech,-1.8*ech],[ech,0],[-ech,0]])
    draw_polygon(P,ax,'blue')    
    plot([   0,   L,  L,  L/2,   L/2,   L/2,  0,  0],
         [-L-d,-L-d, -d,   -d,   2-d,    -d, -d,-L-d],'black',linewidth=3)
    b=-x[2]     
    P=array([[0,-L-d+L],[L,-L-d+L],[L,-L/2-L*b/2-d],[0,-L/2-L*b/2-d]])
    draw_polygon(P,ax,'white')     
    plot([-ech,ech],[-d0,-d0],'red',linewidth=1)    
  

def f(x,u):
    x=x.flatten()
    d,v,b=x[0],x[1],x[2]
    dv = g-(g*max(0,L+min(d,0))+v*abs(v)*cx/2)/(1+0.1*b)
    return array([  [v],
                    [dv],
                    [u]])

def control(x,d0,d0dot,d0ddot):
    x=x.flatten()
    d,v,b=x[0],x[1],x[2]
    u=sign(d0ddot-(g-(g*L+v*abs(v)*cx/2)/((1+0.1*b)*L))+2*(d0dot-v)+(d0-d))
    return u

dt=0.05   
ech=5
p,g,cx = 1000, 9.81, 1.05
x = array([[3],[0],[-1]])       # d,v,b
L=1 #length of the cube
ax=init_figure(-ech,ech,-1.8*ech,0.2*ech)
for t in arange(0,10,dt):
    # d0=5
    # d0dot=0
    # d0ddot=0

    d0=3+sin(0.5*t)
    d0dot=0.5*cos(0.5*t)
    d0ddot=-0.25*sin(0.5*t)

    u=control(x,d0,d0dot,d0ddot)
    
    x=x+dt*f(x,u)
    if abs(x[2])>1:
        x[2]=sign(x[2])
    draw_buoy(x)
pause(3)