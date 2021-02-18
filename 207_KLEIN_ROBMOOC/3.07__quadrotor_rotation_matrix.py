from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
fig = figure()
ax = Axes3D(fig)

m = 10
g = 9.81
b = 2
d = 1
l = 1
I=array([   [10,0,0],               # matrice d'inertie
            [0,10,0],
            [0,0,20]])
dt = 0.01  
B=array([   [b,b,b,b],
            [-b*l,0,b*l,0],
            [0,-b*l,0,b*l],
            [-d,d,-d,d]])

def draw_quadri(x): # vecteur d'état x,y,z, angles d'Euler
    # ax.clear()
    clean3D(ax,-30,30,-30,30,0,30)
    draw_quadrotor3D(ax,x,α,5*l)    # we infate the robot, just to see something

def f(x,w):
    x=x.flatten()
    φ,θ,ψ=x[3:6]
    R   = eulermat(φ,θ,ψ)                   # matrice de rotation
    vr  = (x[6:9]).reshape(3,1)
    wr  = (x[9:12]).reshape(3,1)
    w2  = w*abs(w)                           
    τ   = B@w2.flatten()  
    dp  = R@vr 
    dR  = R@expm(adjoint(wr))
    dvr = -adjoint(wr)@vr+inv(R)@array([[0],[0],[g]])+array([[0],[0],[-τ[0]/m]])
    dφθψ= eulerderivative(φ,θ,ψ) @ wr
    dwr = inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1)) 
    return  vstack((dp,dφθψ,dvr,dwr))

def f_vdp(x):
	x   = x.flatten()
	vdp0= x[1]
	vdp1= -(0.001*(x[0]**2)-1)*x[1]-x[0]
	dX  = array([[vdp0], [vdp1]])
	return dX

def control(X):    
    X = X.flatten()
    x,y,z,φ,θ,ψ = list(X[0:6])
    vr = (X[6:9]).reshape(3,1)
    wr = (X[9:12]).reshape(3,1)
    R = eulermat(φ,θ,ψ)
    dp = R@vr

    # --4--
    zd = -10
    vd = 10
    fd = f_vdp(array([[x], [y]]))
    τd0 = 300*tanh(z-zd) + 60*vr[2]
    φd = 0.5*tanh(10*sawtooth(angle(fd)-angle(dp))) # 10 de precision du cap
    θd = -0.3*tanh(vd-vr[0])[0]
    ψd = angle(dp)
    Rd =eulermat(φd,θd,ψd)

    #inverso of block 3
    wrd = 5*R.T@adjoint_inv( logm(Rd @ R.T) )

    #inverso of block 2
    τd13 = I@( (100*(wrd-wr)) + adjoint(wr)@I@wr)

    #inverse of block 1
    W2 = inv(B)@vstack(([τd0], τd13))
    w = sqrt(abs(W2))*sign(W2)
    return w

x = array([[0,0,-5,                                 # x,y,z
                    1,0,0,                          # euler angles
                            10,0,0,                 # vr
                                    0,0,0]]).T      # wr (front,right,down)

α=array([[0,0,0,0]]).T                              # angles for the blades

for t in arange(0,10,dt):
    w=control(x)
    xdot=f(x,w)
    x  = x + dt*xdot
    draw_quadri(x)
    α=α+dt*50*w     # euler 
    pause(0.001)
pause(1)    