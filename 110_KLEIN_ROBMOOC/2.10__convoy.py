from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    x,u=x.flatten(),u.flatten()
    xdot = array([
                [x[3]*cos(x[2])],
                [x[3]*sin(x[2])],
                [u[0]],
                [u[1]],
                [x[3],
                ]])
    return(xdot)

def control(x,w,dw):
    """
    Controleur - linearisation par bouclage. 
    Approximation car pas de ddw qui est necessaire pour la convergence theorique. 
    """
    x=x.flatten()
    A=array([
        [-x[3]*sin(x[2])   ,   cos(x[2])],
        [ x[3]*cos(x[2])    ,   sin(x[2])]])
    y=array([
        [x[0]],
        [x[1]]])
    dy=array([
        [x[3]*cos(x[2])],
        [x[3]*sin(x[2])]])
    u=inv(A)@((w-y)+(dw-dy))
    return u    
    
    
ax=init_figure(-30,30,-30,30)
xa  = array([[10],[0],[1],[1],[0]])
m   = 6                                                         # nombre de voitures
X=array([4*arange(0,m),zeros(m),ones(m),3*ones(m),zeros(m)])    # vecteur d'etat pour chacun des robots
Lx,Ly = 20,5                                                    # distance entre les robots
e   = np.linspace(0.,2*pi,30)
p   = array([[Lx*cos(e)],[Ly*sin(e)]])
S   = zeros((5,1))
dt  = 0.05
ds  = .1
d   = 5

for t in arange(0,10,dt):
    clear(ax)
    wa  = array([
        [Lx*sin(0.1*t)],
        [Ly*cos(0.1*t)]])
    dwa = array([
        [Lx*0.1*cos(0.1*t)],
        [-Ly*0.1*sin(0.1*t)]])
    if xa[4]>ds:
        S=append(S,xa)
        xa[4]=0                                                 # on remet le compteur metrique a 0
    ua  = control(xa,wa,dwa)    
    plot(wa[0][0],wa[1][0],'ro')
    plot(p[0][0],p[1][0])
    draw_tank(xa,'blue')                                        # leader
    xa  = xa + dt*f(xa,ua)
    for i in range(m):
        j=size(S,0)-d*i/ds   ### PROBLEME A RESOUDRE
        if j>0:
            xai=S[:,j]
            wi=xai[0:2]
            dwi=xa[3]*array([
                            [cos(xai[2])],
                            [sin(xai[2])]])
            ui = control(X[:,i],wi,dwi)
            # plot(wi[0,0],wi[1,0],"blue")
        else:
            ui = array([[0.2],[0]])
        x=X[:,i].reshape(5,1)
        draw_tank(x,'black')
        x=x+f(x,ui)*dt        
        X[:,i]  = x.flatten()            
pause(1)