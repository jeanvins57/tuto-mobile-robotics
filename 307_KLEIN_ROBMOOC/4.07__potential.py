from roblib import *

def f(x,u):
	x,u  = x.flatten(), u.flatten()
	v,θ = x[2],x[3]	
	return array([[v*cos(θ)],
                  [v*sin(θ)],
                  [u[0]],
                  [u[1]]])
	
def draw_field(phat,qhat,vhat):
	Mx	= arange(xmin,xmax,0.3)
	My	= arange(ymin,ymax,0.3)
	X1,X2 = meshgrid(Mx,My)
	Nq1 = X1 - qhat[0]
	Nq2 = X2 - qhat[1]
	VX = 0*X1 + vhat[0] - 2*(X1-phat[0]) + Nq1/((Nq1**2 + Nq2**2)**(3/2))
	VY = 0*X2 + vhat[1] - 2*(X1-phat[1]) + Nq2/((Nq1**2 + Nq2**2)**(3/2))
	R = sqrt(VX**2+VY**2)
	quiver(Mx,My,VX/R,VY/R)

x = array([[10],[0],[0],[0]])  # x,y,v,θ
dt = 0.05
xmin,xmax,ymin,ymax = 0,10,0,10
ax = init_figure(xmin,xmax,ymin,ymax)

for t in arange(0,10,dt):
	clear(ax)
	# phat = array([[t],[t]])
	phat = array([[8],[8]])
	qhat = array([[4.5+0.1*t],[5]]) 
	vhat = array([[1],[1]])
	draw_disk(qhat,0.1,ax,"magenta")
	draw_disk(phat,0.1,ax,"green")
	
	nq = x[:2] - qhat
	w = vhat - 2*(x[:2] - phat) + nq/norm(nq)**3
	vbar = norm(w)
	thetabar = arctan2(w[1, 0], w[0, 0])
	
	u = array([[vbar - x[2, 0]], [10*arctan(tan(0.5*(thetabar - x[3, 0])))]])
	x = x + dt*f(x,u)	
	draw_tank(x[[0,1,3]],'red',0.2) # x,y,θ
	draw_field(phat,qhat,vhat)
pause(5)