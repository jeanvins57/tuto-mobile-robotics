## ROBMOOC - 2020 
## Jean-Vincent KLEIN
## FISE 2022
# p35

from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    xr,yr,θr,vr=x.flatten()
    u1,u2=u.flatten()
    return (array([[vr*cos(θr)],[vr*sin(θr)],[u1],[u2]]))

def control(x,w,dw):
    A = array([
        [-x[3,0]*sin(x[2,0]),cos(x[2,0])],
        [x[3,0]*cos(x[2,0]),sin(x[2,0])]
        ])
    y = array([
        [x[0,0]],
        [x[1,0]]
        ])
    dy = array([
        [x[3,0]*cos(x[2,0])],
        [x[3,0]*sin(x[2,0])]
        ])
    v=(w-y)+2*(dw-dy)
    u=inv(A)@v
    return u 

ax=init_figure(-30,30,-30,30)
dt = 0.1
xa = array([[10],[0],[1],[1]]) # x,y,theta,v
xb = array([[0],[0],[1],[2]])
xc = array([[-10],[0],[1],[2]])
ua = array([[0],[0]])
Lx = 15
Ly = 7
l = 6

for t in arange(0,50,dt) :
    clear(ax)
    
    wa = array([
        [Lx*sin(0.1*t)],
        [Ly*cos(0.1*t)] ])
    dwa = array([
        [Lx*0.1*cos(0.1*t)],
        [-Ly*0.1*sin(0.1*t)] ])
    ua = control(xa,wa,dwa)
    draw_tank(xa)
    xa = xa+dt*f(xa,ua)

    
    # Voiture B
    wb = array([
        [xa[0,0]-l*cos(xa[2,0])],
        [xa[1,0]-l*sin(xa[2,0])]
    ])
    dwb = array([
        [xa[3,0]*cos(xa[2,0]) + l*ua[0,0]*sin(xa[2,0])],
        [xa[3,0]*sin(xa[2,0]) - l*ua[0,0]*sin(xa[2,0])]
    ])
    ub = control(xb,wb,dwb)
    draw_tank(xb,col="red")
    xb = xb+dt*f(xb,ub)

        
    # Voiture C
    wc = array([
        [xb[0,0]-l*cos(xb[2,0])],
        [xb[1,0]-l*sin(xb[2,0])]
    ])
    dwc = array([
        [xb[3,0]*cos(xb[2,0]) + l*ub[0,0]*sin(xb[2,0])],
        [xb[3,0]*sin(xb[2,0]) - l*ub[0,0]*sin(xb[2,0])]
    ])
    uc = control(xc,wc,dwc)
    draw_tank(xc,col="green")
    xc = xc+dt*f(xc,uc)
    
pause(1)

"""
Observations : pas de gestion des collisions
La voiture C ne suit pas exactement la direction donnée à la voiture A
La voiture C suit la voiture B qui suit la voiture A. on 
a une accumulation des incertitudes (surtout dans les virages). 

Un superviseur qui envoie au voitures les consignes de
maniére centralisée permettrait de limiter cet
effet de propagation des incertitude.
"""