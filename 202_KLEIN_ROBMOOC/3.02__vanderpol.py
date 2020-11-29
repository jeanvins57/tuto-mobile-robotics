from roblib import *  

def f(x,u):
	x,u  = x.flatten(), u.flatten()
	xdot = array([[x[3]*cos(x[4])*cos(x[2])],
				  [x[3]*cos(x[4])*sin(x[2])],
				  [x[3]*sin(x[4])/3],
				  [u[0]],
				  [u[1]]])
	return xdot

def draw_field0(xmin,xmax,ymin,ymax):
	Mx	= arange(xmin,xmax,2)
	My	= arange(ymin,ymax,2)
	X1,X2 = meshgrid(Mx,My)
	VX	= X2
	VY	= -(0.01*(X1**2)-1)*X2-X1
	VX	= VX/sqrt(VX**2+VY**2)
	VY	= VY/sqrt(VX**2+VY**2)
	quiver(Mx,My,VX,VY)

x = array([[0],[5],[pi/2],[5],[0.5]])
dt = 0.05
ax = init_figure(-40,40,-30,30)
for t in arange(0,5,dt):
	clear(ax)
	draw_field0(-40,40,-40,40)
	draw_car(x)
	vdp = array([[x[1,0]], [-(0.01*(x[0,0]**2)-1)*x[1,0]-x[0,0]]])
	w = array([[10], [angle(vdp)]])
	ubar = array([[w[0,0]],[3*sawtooth(w[1,0]-x[2,0])]])
	u = 10*(ubar-array([[x[3,0]*cos(x[4,0])], [x[3,0]*sin(x[4,0])/3]]))
	x = x + dt*f(x,u)

