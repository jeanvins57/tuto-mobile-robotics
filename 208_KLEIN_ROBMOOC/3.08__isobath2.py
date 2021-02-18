from roblib import *  
		
def h(x,y):
	"""
	Seafloor equation.
	"""
	return 2*exp(-((x+2)**2+(y+2)**2)/10) + 2*exp(-((x-2)**2+(y-2)**2)/10) - 10

def draw_mesh():
	"""
	Drawing seabed.
	"""
	Mx=arange(-L,L,1)
	X,Y = meshgrid(Mx,Mx)
	H = h(X,Y)
	# ax.plot_surface(X,Y,H)
	ax.contour(X,Y,H)
	return()

def f(x,u):
	"""
	State function. 
	"""
	x1,x2,x3,ψ = x.flatten()
	u1,u2 = u.flatten()
	return array([	[cos(ψ)],
					[sin(ψ)],
					[u1],
					[u2]])

def gradh(x,y):
	"""
	Defining gradient function.
	"""
	ep = 0.1
	return array([(h(x+ep,y)-h(x,y))/ep, (h(x,y+ep)-h(x,y))/ep])

def g(x):
	"""
	"""
	x = x.flatten()
	ψ = x[3]
	return array([[x[2]-h(x[0],x[1])],
					[angle(gradh(x[0],x[1]))-	ψ],
					[-x[2]]])

def control(y):
	"""
	Defining controler. 
	"""
	h0 = -9 # required
	y0 = 2 # required
	y = y.flatten()
	u1 = 0.5*(y[2]-y0) # coefficient k = 0.5
	u2 = -tanh(h0+y[2]+y[0]) + sawtooth(y[1]+pi/2) # possible to use PID approach (proportional–integral–derivative)
	return array([[u1],[u2]])

ax = Axes3D(figure())
x = array([[2],[-1],[-1],[0]]) # x,y,z,ψ
L = 10 #size of the world
dt = 0.1

for t in arange(0,100,dt):
	y = g(x)
	u = control(y)
	x = x + dt*f(x,u)
	draw_mesh()
	if t%2==0:
		draw_robot3D(ax,x[0:3],eulermat(0,0,x[3,0]),'blue',0.1)
	# pause(0.01)

waitforbuttonpress()