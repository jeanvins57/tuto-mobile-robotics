from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
from numpy import concatenate

fig = figure()
ax = Axes3D(fig)
m,g,b,d,l=10,9.81,2,1,1
I=array([[10,0,0],[0,10,0],[0,0,20]])
dt = 0.05

def draw_hexarotor3D(ax,p,R,α,col):
    lz=5*l
    Ca=hstack((circle3H(0.3*lz),[[0.3*lz,-0.3*lz],[0,0],[0,0],[1,1]])) # the disc + the blades
    Ca=expwH([0,pi/2,0])@Ca
    T = tran3H(*p) @ ToH(R)
    for i in range(0,N):
        ai=pi/2*adjoint([[1],[0],[0]])@D[:,i]
        Ci=T @ tran3H(*(lz*Q[:,i]))@expwH(ai)@eulerH(α[i],0,0)@Ca
        draw3H(ax, Ci, col[i], True, -1)
    M = T @ add1([[lz,-lz,0,0, 0],[0,0,0,lz,-lz],[0,0,0,0,0]])
    draw3H(ax,M,'grey',True,-1)

def draw_platform(ax,p,R):
    lz=5*l
    Ca=expwH([0,pi/2,0])@circle3H(0.3*lz)
    T = tran3H(*p) @ ToH(R)
    for i in range(0,N-2):
        ai=pi/2*adjoint([[1],[0],[0]])@D[:,i]
        Ci=T @ tran3H(*(lz*Q[:,i]))@expwH(ai)@Ca
        draw3H(ax, Ci, 'black', True, -1)
    M = T @ add1([[lz,-lz,-lz, lz,lz],[lz,lz,-lz,-lz,lz],[0,0,0,0,0]])
    draw3H(ax,M,'grey',True,-1)

def clock_hexa(p,R,vr,wr,f):
    fr_τr = C@f
    fr = fr_τr[0:3,0]
    τr = fr_τr[3:7]
    p=p+dt*R@vr 
    vr=vr+dt*(-adjoint(wr)@vr+inv(R)@array([[0],[0],[g]]) + array([[fr[0]],[fr[1]],[fr[2]]])/m)
    R=R@expm(adjoint(dt*wr)) 
    wr=wr+dt*(inv(I)@(-adjoint(wr)@(I@wr)+ τr))
    return p,R,vr,wr

def control(p,R,vr,wr,Rd,pd,dRd,ddRd,C):
    
    dpd = array([[0],[0],[0]])
    ddpd = array([[0],[0],[0]])

    def positioner(R,vr,wr,Rd,pd,dpd,ddpd):
        dvrd = inv(R)@ddpd + 2*(inv(R)@dpd -vr) + inv(R)@(pd-p) - adjoint(wr)@vr 
        return dvrd

    def orientator(Rd,dRd,ddRd,R,wr):
        wd = adjoint_inv(dRd@inv(Rd))                                         
        dwd = adjoint_inv(dRd@dRd.T + ddRd@inv(Rd)) 
        er = inv(R)@adjoint_inv(logm(Rd@inv(R)))
        der = - wr + inv(Rd)@wd
        dwrd = (adjoint(wr)@inv(Rd) + dRd.T)@wd + inv(Rd)@dwd + 2*der + 1*er
        return dwrd

    def inverse_block(dwrd,dvrd):
        frd = m*dvrd -m*inv(R)@array([[0],[0],[g]])- m*adjoint(wr)@vr          
        τrd = I@dwrd + adjoint(wr)@(I@wr)
        return vstack((frd,τrd))
    
    dwrd = orientator(Rd,dRd,ddRd,R,wr)
    dvrd = positioner(R,vr,wr,Rd,pd,dpd,ddpd)
    frd_τrd = inverse_block(dwrd,dvrd)         

    return inv(C)@frd_τrd


def pd(t):  return  array([[sin(0.3*t)], [cos(0.4*t)], [-10+0.1*sin(0.3*t)]])
def Rd(t):  return  expw([[sin(t)], [cos(2*t)], [t]])
def dRd(t): return (1/(2*dt))*(Rd(t+dt)-Rd(t-dt))
def ddRd(t): return (1/(2*dt))*(dRd(t+dt)-dRd(t-dt))

Q=array([[0,-l, 0, l,l/2, 0],[l, 0,-l, 0,  0,l/2], [0, 0, 0, 0,  0, 0]])  #positions of the rotors, all blades have the same pitch
D=array([[1,0,0,0,1,0],[0,0,0,0,0,1],[0,1,1,1,0,0]])  #orientation of the forces
N=D.shape[1]
Rτ=hstack((adjoint(Q[:, i]) @ D[:, i].reshape(3, 1) for i in range(0, N) ))
C=vstack((D,Rτ))

p = array([[10], [0], [-20]])  #x,y,z (front,right,down)
R = eulermat(0.2,0.3,0.4)
vr = array([[0], [0], [0]])
wr = array([[0], [0], [0]])
α=zeros((N,1))
for t in arange(0,10,dt):

    f = control(p,R,vr,wr,Rd(t),pd(t),dRd(t),ddRd(t),C)
    p, R, vr, wr = clock_hexa(p, R, vr, wr, f)
    clean3D(ax, -20, 20, -20, 20, 0, 25)
    draw_hexarotor3D(ax, p, R, α, ['green','black','red','blue','orange','brown'])
    draw_platform(ax, pd(t), Rd(t))
    α = α + dt * 30 * f
    pause(0.001)

pause(10)


