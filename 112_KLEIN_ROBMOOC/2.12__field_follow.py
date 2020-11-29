from roblib import *  

def draw0(x):
	draw_tank(x,'darkblue',0.3)
	a,b = array([[-30],[0]]), array([[30],[0]])
	draw_segment(a,b,'red',2)

def fhat(x1,x2):
	return cos(-arctan(x2)), sin(-arctan(x2))

def vdp(x1,x2): 
	return x2,-(0.01*(x1**2)-1)*x2-x1

def f(x,u):
	θ = x[2,0]
	return array([[cos(θ)], [sin(θ)], [u]])

def control(x):
	x = x.flatten()
	u = -x[2] - arctan(x[1]) - sin(x[2])/(1+x[1]**2)
	return u

def control_vdp(x):
	x = x.flatten()
	x1,x2,x3 = x[0], x[1], x[2]
	a,b = vdp(x1,x2)
	da = sin(x3)
	db = -0.02*x1*x2*cos(x3) - (0.01*x1**2-1)*sin(x3) - cos(x3)
	y = sawtooth(x3-arctan2(b,a))
	bx = (b*da-a*db)/(a**2+b**2)
	u = -y-bx
	return u

x = array([[-2], [-3], [1]])
# x=x,y,theta
dt = 0.1
ech = 30
ax = init_figure(-ech,ech,-ech,ech)
for t in arange(0,500,dt):
	clear(ax)
	draw0(x)
	draw_field(ax,vdp,-ech,ech,-ech,ech,1)
	u = control_vdp(x)
	x = x+dt*f(x,u)
pause(10)
