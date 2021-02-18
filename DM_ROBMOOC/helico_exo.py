from roblib import *
fig = figure()
ax = Axes3D(fig)

m,g,l=10,9.81,2
ω1,ω2=100,100
β1,β4=0.02,0.002
β2,β3=β1/10,β1/10
δ1=β1/5
B=array([[β1*ω1**2,0,0,0],[0,β2*ω1**2,0,0],[0,0,β3*ω1**2,0],[-δ1*ω1**2,0,0,-β4*l*ω2**2]])
I=array([[10,0,0],[0,20,0],[0,0,20]])
dt = 0.01

def draw_helico3D(ax,p,R,α,l):
    Ca1=hstack((circle3H(0.7*l),[[0.7*l,-0.7*l],[0,0],[0,0],[1,1]])) # the disc + the blade
    Ca2=hstack((circle3H(0.2*l),[[0.2*l,-0.2*l],[0,0],[0,0],[1,1]])) # the disc + the blades
    T = tran3H(*p) @ ToH(R)
    C1= T @ tran3H(0,0,-l/4) @eulerH(0,0,-α[0])@Ca1
    C2= T @ tran3H(-l,0,0)@eulerH(pi/2,0,0) @eulerH(0,0,α[1])@Ca2
    M = T @ add1(array([[-l,0,0],[0,0,0],[0,0,-l/4]]))
    draw3H(ax,M,'black',True,-1)  #body
    draw3H(ax, C2, 'green', True,-1)
    draw3H(ax, C1, 'blue', True, -1)

def clock_helico(p,R,vr,wr,u):
    τ=B@u.flatten()
    p=p+dt*R@vr
    vr=vr+dt*(-adjoint(wr)@vr+inv(R)@array([[0],[0],[g]])+array([[0],[0],[-τ[0]/m]]))
    R=R@expm(adjoint(dt*wr))
    wr=wr+dt*(inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1)))
    return p,R,vr,wr

def f_vdp(x):
    x = x.flatten()
    vdp0 = x[1]
    vdp1 = -(0.001*(x[0]**2)-1)*x[1]-x[0]
    dX = array([[vdp0], [vdp1]])
    return dX

def control(p,R,vr,wr):
    dp = R@vr
    x,y,z=list(p)
    φ,θ,ψ=eulermat2angles(R)
    print(φ,θ,ψ)

    # -- Staight fwd, stable --
    # zd = -10
    # τd0 = 300*tanh(z-zd) + 60*vr[2]
    # φd = 0
    # θd = 0
    # ψd = angle(dp)

    # -- Looping not stable --
    zd = -100
    τd0 = 1000*tanh(z-zd) + 200*vr[2]
    φd = 0 
    θd = angle(dp)
    ψd = angle(dp)

    # -- VDP direction not stable = Gimbal Lock? --
    # zd = -50
    # vd = 250
    # fd = f_vdp(array([[x], [y]]))
    # τd0 = 1000*tanh(z-zd) + 200*vr[2]
    # φd = 0.5*tanh(10*sawtooth(angle(fd)-angle(dp))) # 10 de precision du cap
    # θd = -0.3*tanh(vd-vr[0])[0]
    # ψd = angle(dp)

    #inverse of block 3
    wrd = 10*inv(R)@array([[sawtooth(φd-φ)], [sawtooth(θd-θ)], [sawtooth(ψd-ψ)]])
    #inverse of block 2
    τd13 = I@( (100*(wrd-wr)) + adjoint(wr)@I@wr)
    #inverse of block 1
    W2 = inv(B)@vstack(([τd0], τd13))
    w = sqrt(abs(W2))*sign(W2)
    return w

# Initialisation
p = array([[0],[0],[-10]])
R = eulermat(0.4,0.2,0.4)
vr = array([[13], [0], [0]])
wr = array([[0], [0], [0]])
α=array([[0,0]]).T
t1=0

for t in arange(0,1000,dt):
    clean3D(ax, -20, 20, -20, 20, 0, 40)
    # u = array([[0],[0],[0],[0]])
    u = control(p,R,vr,wr)
    p, R, vr, wr = clock_helico(p, R, vr, wr, u)
    draw_helico3D(ax,p,R,α,5*l)
    pause(0.001)
    α = α + dt * 30 * array([[ω1],[ω2]])
pause(10)