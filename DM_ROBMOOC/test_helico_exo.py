# Jean-Vincent KLEIN
# FISE 2022

from roblib import *
fig = figure()
ax = Axes3D(fig)

m,g,l=10,9.81,2                 # int pb
ω1,ω2=100,100                   # rotors rotational speed
β1,β4=0.02,0.002                #
β2,β3=β1/10,β1/10               #
δ1=β1/5                         #

B=array([                       # 
    [β1*ω1**2,0,0,0],  
    [0,β2*ω1**2,0,0],
    [0,0,β3*ω1**2,0],
    [-δ1*ω1**2,0,0,-β4*l*ω2**2]])

I=array([                       # inertia matrix
    [10,0,0],
    [0,20,0],
    [0,0,20]])

dt = 0.01

def draw_helico3D(ax,p,R,α,l):
    clean3D(ax, -20, 20, -20, 20, 0, 40)
    Ca1=hstack((circle3H(0.7*l),[[0.7*l,-0.7*l],[0,0],[0,0],[1,1]])) # the disc + the blade
    Ca2=hstack((circle3H(0.2*l),[[0.2*l,-0.2*l],[0,0],[0,0],[1,1]])) # the disc + the blades
    T = tran3H(*p) @ ToH(R)
    C1= T @ tran3H(0,0,-l/4) @eulerH(0,0,-α[0])@Ca1
    C2= T @ tran3H(-l,0,0)@eulerH(pi/2,0,0) @eulerH(0,0,α[1])@Ca2
    M = T @ add1(array([[-l,0,0],[0,0,0],[0,0,-l/4]]))
    draw3H(ax,M,'black',True,-1)  #body
    draw3H(ax, C2, 'green', True,-1)
    draw3H(ax, C1, 'blue', True, -1)

# def clock_helico(p,R,vr,wr,u):
#     τ=B@u.flatten()
#     p=p+dt*R@vr
#     vr=vr+dt*(-adjoint(wr) @ vr+inv(R) @ array([[0],[0],[g]]) + array([[0],[0],[-τ[0]/m]]))
#     R=R@expm(adjoint(dt*wr))
#     wr=wr+dt*(inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1)))
#     return p,R,vr,wr

def f(x,w): # added
    x=x.flatten()
    φ,θ,ψ   = x[3:6]
    R       = eulermat(φ,θ,ψ)                   # rotation matrix
    vr      = (x[6:9]).reshape(3,1)
    wr      = (x[9:12]).reshape(3,1)
    w2      = w*abs(w)                           
    τ       = B @ w2.flatten()                  # total thrust
    dp      = R @ vr 
    dR      = R @ expm(adjoint(wr))
    dvr     = -adjoint(wr) @ vr+inv(R) @ array([[0],[0],[g]]) + array([[0],[0],[-τ[0]/m]])
    dφθψ    = eulerderivative(φ,θ,ψ) @ wr
    dwr     = inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1)) 
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

    zd = -10
    vd = 10
    fd = f_vdp(array([[x], [y]]))
    τd0 = 300*tanh(z-zd) + 60*vr[2]
    φd = 0.5*tanh(10*sawtooth(angle(fd)-angle(dp))) # 10 de precision du cap
    θd = -0.3*tanh(vd-vr[0])[0]
    ψd = angle(dp)
    Rd =eulermat(φd,θd,ψd)

    #inverse of block 3
    wrd = 5*R.T@adjoint_inv( logm(Rd @ R.T) )

    #inverse of block 2
    τd13 = I @ ((100*(wrd-wr))+adjoint(wr)@I@wr)

    #inverse of block 1
    W2 = inv(B)@vstack(([τd0], τd13))
    w = sqrt(abs(W2[0:2]))*sign(W2[0:2])
    print(W2[0:2])
    return w

# Construction du vecteur d'etat 
x = array([[0,0,-10,                                # p = x,y,z
                    1,0,0,                          # euler angles φ,θ,ψ
                            10,0,0,                 # vr
                                    0,0,0]]).T      # wr (front,right,down)
α=array([[0,0]]).T                                  # angles for the blades
t1=0

# for t in arange(0,2,dt):
#     # u=array([[0],[0],[0],[0]])
#     u=control(p,R,vr,wr)
#     p, R, vr, wr = clock_helico(p, R, vr, wr, u)
#     draw_helico3D(ax,p,R,α,5*l)
#     pause(0.001)
#     α = α + dt * 30 * array([[ω1],[ω2]])
# pause(10)

for t in arange(0,10,dt):
    w       = control(x)
    xdot    = f(x,w)
    x       = x + dt*xdot
    # print (x)
    α       = α+dt*50*w     # euler 
    # draw_helico3D(ax,p,R,α,5*l)
    pause(0.001)
pause(1)    