from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    x    = x.flatten()
    return array([[5*cos(x[2])],[5*sin(x[2])],[u]])

def control(x):
	alpha = arctan2(x[1][0],x[0][0])
	phi = pi + x[2][0] - alpha
	if (cos(phi)<sqrt(2)/2):
		u = 1
	else : 
		u = -sin(phi)
	return u

x    = array([[15],[20],[1]])
dt   = 0.1
ax=init_figure(-30,30,-30,30)
for t in arange(0,30,dt):
    clear(ax)
    draw_disk(array([[0],[0]]),10,ax,'cyan')
    u = control(x)
    draw_tank(x,'red')
    if (u==1):
        draw_tank(x,'blue')
    else : draw_tank(x,'green')
    x = x+dt*f(x,u)+0.03*randn(3,1)         

