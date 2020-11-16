## ROBMOOC - 2020 
## Jean-Vincent KLEIN
## FISE 2022
# p35

from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def draw_crank(x): 
    global θ1
    global θ2
    θ1 = x[0, 0]
    θ2 = x[1, 0]
    z=L1*array([[cos(θ1)],[sin(θ1)]])
    y=z+L2*array([[cos(θ1+θ2)],[sin(θ1+θ2)]])
    plot( [0,z[0,0],y[0,0]],[0,z[1,0],y[1,0]],'magenta', linewidth = 2)   
    draw_disk(c,r,ax,"cyan")
    return(y)

def f(x):
    θ1=x[0,0]
    θ2=x[1,0]
    dθ1=1
    dθ2=2
    return(array([[dθ1],[dθ2]]))
    
L1,L2 = 4,3
c = array([[1],[2]]) # en rad/s
r=4
dt = 0.01
x = array([[-1],[1]])
c=array([[1],[2]])

ax=init_figure(-8,8,-8,8)

for t in arange(0,10,dt) :
    clear(ax)
    y=draw_crank(x)
    w =c+r*array([[cos(t)],[sin(t)]])
    dw=r*array([[-sin(t)],[cos(t)]])
    v=w-y+dw
    # A=array([   [float(-y[1]), -L2*sin(float(x[0])+float(x[1]))],
                # [float(y[0]),  L2*cos(float(x[0])+float(x[1]))] ])
    A = array([ [-y[1, 0]   ,   -L2*sin(θ1+θ2)      ],
                [y[0, 0]    ,   L2 * cos(θ1+θ2)   ]])
    u=inv(A)@v
    x=x+u*dt