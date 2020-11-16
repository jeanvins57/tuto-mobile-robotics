## ROBMOOC - 2020 
## Jean-Vincent KLEIN
## FISE 2022


from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def draw_pools(x):
    x=x.flatten()
    plot([0,0],[10,1],'black',linewidth=2)    
    plot([-7,23],[0,0],'black',linewidth=5)    
    plot([16,16],[1,10],'black',linewidth=2)    
    plot([4,4,6,6],[10,1,1,10],'black',linewidth=2)    
    plot([10,10,12,12],[10,1,1,10],'black',linewidth=2)    
    P=array([[0,x[0]],[0,1],[-6,0],[22,0],[16,1],[16,x[2]],[12,x[2]],[12,1]
            ,[10,1],[10,x[1]],[6,x[1]],[6,1],[4,1],[4,x[0]]])
    draw_polygon(P,ax,'blue')       
    P=array([[1,10],[1,x[0]],[1+0.1*u[0],x[0]],[1+0.1*u[0],10]])            
    draw_polygon(P,ax,'blue')
    P=array([[13,10],[13,x[2]],[13+0.1*u[1],x[2]],[13+0.1*u[1],10]])            
    draw_polygon(P,ax,'blue')

def q(h):
    a=0.4
    g=9.81
    return a*sign(h)*sqrt(2*g*abs(h))

def f(x,u):
    xdot=array([[-q(x[0,0])-q(x[0,0]-x[1,0])+u[0,0]],
                [ q(x[0,0]-x[1,0]) - q(x[1,0]-x[2,0]) ],
                [-q(x[2,0]) + q(x[1,0]-x[2,0]) + u[1,0]] ])
    return xdot

dt = 0.05
x = array([[2],[2],[2]])
u = array([[2],[10]])
ax=init_figure(-10,25,-2,12)
z = array([[0],[0]])

for t in arange(0,10,dt) :
    w=array([[7],[3]])
    dw=array([[0],[0]])
    y=array([[x[0,0]],[x[2,0]]])
    b=array([[-q(x[0,0])-q(x[0,0]-x[1,0])],[-q(x[2,0]) + q(x[1,0]-x[2,0])]])
    v=z+2*(w-y)+dw
    u=v-b
    clear(ax)
    draw_pools(x)
    z=z+(w-y)*dt
    x = x + dt*f(x,u)
print(x)

"""
Observation : si on ajoute une peertubation (ex: fuite), l'effet intégrateur permet de la corriger
Attention cela est possible seulement pour une perturbation constante. (dw ne doit pa dépendre du temps par exemple)
"""