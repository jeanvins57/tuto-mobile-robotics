from roblib import *

def f(x,u):
	x = x.flatten()
	θ = x[2]
	return array([[cos(θ)],[sin(θ)],[u]])
	
x = array([[0],[0],[0]]) # x,y,theta
dt = 0.1
ax = init_figure(-10,10,-10,10)

for t in arange(0,30,dt):
	clear(ax)
	ax.text(2,6, r"hello world! %i"%t, fontsize=15)
	thetabar = np.pi/2 + 10*np.pi
	thetat = thetabar - x[2, 0]
	#u = np.mod((thetat + np.pi), 2*np.pi) - np.pi	  # delta in [-pi, pi]
	#u = np.mod((thetat), 2*np.pi)					  # delta in [0, 2*pi]
	u = np.mod(thetat, 2*np.pi) - 2*np.pi			  # delta in [-2*pi, 0]
	x = x + dt*f(x,u)	
	draw_tank(x,'red',0.3) 
	