from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
fig = figure()
ax = Axes3D(fig)

m,g,b,d,l=10,9.81,2,1,1
I=array([[10,0,0],[0,10,0],[0,0,20]])
dt = 0.01  
B=array([[b,b,b,b],[-b*l,0,b*l,0],[0,-b*l,0,b*l],[-d,d,-d,d]])

def draw_quadri(x): # vecteur d'état x,y,z, angles d'Euler
    ax.clear()
    clean3D(ax,-30,30,-30,30,0,30)
    draw_quadrotor3D(ax,x,α,5*l)    # we infate the robot, just to see something
         
        
def f(x,w):
    x=x.flatten()
    φ,θ,ψ=x[3:6]
    vr=(x[6:9]).reshape(3,1)
    wr=(x[9:12]).reshape(3,1)
    w2=w*abs(w)
    τ=B@w2.flatten()
    E=eulermat(φ,θ,ψ)
    dp=E@vr
    dvr=-adjoint(wr)@vr+inv(E)@array([[0],[0],[g]])+array([[0],[0],[-τ[0]/m]])  
    dφθψ= eulerderivative(φ,θ,ψ) @ wr
    dwr= inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1))            
    return  vstack((dp,dφθψ,dvr,dwr))

    
def control(x):    
    return array([[6],[5],[5],[5]]) 

x = array([[0,0,-5,0,0,0,10,0,0, 0,0,0]]).T  #x,y,z,   φ,θ,ψ   vr  wr (front,right,down)
α=array([[0,0,0,0]]).T #angles for the blades

            
for t in arange(0,0.1*3,dt):
    w=control(x)
    xdot=f(x,w)
    x  = x + dt*xdot
    draw_quadri(x)
    α=α+dt*10*w    
    pause(0.001)
pause(1)    
    
   
