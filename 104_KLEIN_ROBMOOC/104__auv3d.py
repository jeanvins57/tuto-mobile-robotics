from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def draw0(x,w):
    clean3D(ax,-10,10,-10,10,0,20)
    draw_axis3D(ax,0,0,0,eye(3,3),10)
    draw_robot3D(ax,x[0:3],eulermat(*x[4:7,0]),'blue')
    # draw_robot3D(ax,w[0:3],eulermat(*x[4:7,0]),'red', size=0.1)
    ax.scatter(w[0,0],w[1,0],w[2,0],color='magenta')

def f(x,u):
    """
    Equation d'etat
    """
    x,u=x.flatten(),u.flatten()
    v,φ,θ,ψ=x[3],x[4],x[5],x[6]
    cφ,sφ,cθ,sθ,cψ,sψ= cos(φ),sin(φ),cos(θ),sin(θ),cos(ψ),sin(ψ)
    return array([ [v*cθ*cψ],[v*cθ*sψ],[-v*sθ],[u[0]] ,
                    [-0.1*sφ*cθ + tan(θ)*v*(sφ*u[1]+cφ*u[2])] ,
                     [cφ*v*u[1] - sφ*v*u[2]] ,
                     [(sφ/cθ)*v*u[1] + (cφ/cθ)*v*u[2]]])

def control(x,w,dw,ddw):
    """
    Loi de commande
    """
    v,φ,θ,ψ=x[3,0],x[4,0],x[5,0],x[6,0]
    ct=cos(θ)
    st=sin(θ)
    cf=cos(φ)
    sf=sin(φ)
    cp=cos(ψ)
    sp=sin(ψ)
    A1=array([
        [ct*cp  , -v*ct*sp  ,   -v*st*cp],
        [ct*sp  , v*ct*cp   ,   -v*st*sp],
        [-st     ,   0       ,   -v*ct]])
    A2=array([
        [1  , 0    ,        0],
        [0  , -sf/ct   ,  cf/ct],
        [0  , cf       ,    -sf]])
    A=A1@A2
    dp=v*array([[ct*cp],[ct*sp],[-st]])
    p=x[0:3,:]
    u=inv(A)*(0.04*(w-p)+0.4*(dw-dp)+ddw)
    return u

def setpoint(t):
    """
    Genere la consigne
    """
    f1=0.01
    f2=6*f1
    f3=3*f1
    R=20
    w=R*array([
        [sin(f1*t)+sin(f2*t)],
        [cos(f1*t)+cos(f2*t)],
        [sin(f3*t)]])
    dw=R*array([
        [f1*cos(f1*t)+f2*cos(f2*t)],
        [-f1*sin(f1*t)-f2*sin(f2*t)],
        [f3*cos(f3*t)]])
    ddw=R*array([
        [-f1**2*sin(f1*t)-f2**2*sin(f2*t)],
        [-f1**2*cos(f1*t)-f2**2*cos(f2*t)],
        [-f3**2*sin(f3*t)]])
    return w,dw,ddw

x = array([[0,0,10,15,0,1,0]]).T
print(x.shape)
dt = 0.1
ax = Axes3D(figure())    
for t in arange(0,50,dt):
    w,dw,ddw=setpoint(t)
    u = control(x,w,dw,ddw)
    x = x + dt * f(x,u)
    draw0(x,w)
    pause(0.001)
pause(1)    