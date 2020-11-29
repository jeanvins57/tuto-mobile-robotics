## Jean-Vincent KLEIN 
# FISE 2022

from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    θ=x[2,0]
    return array([[cos(θ)], [sin(θ)],[u]])

x=array([[-20],[-10],[4]])
u=1
dt= 0.1
a,b = array([[-30],[-4]]), array([[30],[6]])
ax=init_figure(-40,40,-40,40)


for t in arange(0,30,dt):
    clear(ax)

    phi = arctan2(b[1,0] - a[1,0], b[0,0] - a[0,0])
    m = array([[x[0,0]], [x[1,0]]])
    ba = b-a
    ma = m-a
    e = det(array([[ba[0,0],ma[0,0]], [ba[1,0],ma[1,0]]]))/norm(b-a)
    thetabar = phi - arctan(e)
    u = arctan(tan((thetabar-x[2,0])/2))

    draw_tank(x,'darkblue')
    plot2D(hstack((a,b)),'red')
    plot2D(a,'ro')
    plot2D(b,'ro')    

    x   = x+dt*f(x,u)