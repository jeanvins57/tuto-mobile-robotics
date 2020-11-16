from roblib import *

def rotate(v, motifs):
    """
    Rotation des coordonnées du motif
    Entrée: v, motifs
    Sortie: Mat(3x3)
    """
    R=expm(adjoint(v))
    A=np.eye(4)
    A[0:3,0:3]=R
    rot = []
    for i in np.transpose(motifs):
        rot.append(np.dot(A, np.transpose(i)))
    return(np.transpose(array(rot)))

def translate(v, motifs):
    """
    Fonction de translation du motif
    Entrée: v, motifs
    Sortie: Mat(3x3)
    """
    A = np.eye(4)
    A[:3,3] = array(v)
    trans = []
    for i in np.transpose(motifs):
        trans.append(np.dot(A, np.transpose(i)))
    return(np.transpose(array(trans)))

def draw2(motif):
    """
    Trace le motif avec ses transformations successives
    Entrée : motif
    Sortie : N/A -> plot
    """
    plot3D(ax,motif,"black",2)
    cpy = np.copy(motif)
    cpy[2,:] *= 0
    plot3D(ax,cpy,"blue",2)

def f(x, U):
    """
    Equations d'état du système
    Entrées : x, U
    Sortie: X (7x1)
    """
    X = np.array([  x[3]*np.cos(x[5])*np.cos(x[6]),
                    x[3]*np.cos(x[5])*np.sin(x[6]),
                    -x[3]*np.sin(x[5]),
                    U[0],
                    -0.1 * np.sin(x[4]) * np.cos(x[5]) + np.tan(x[5]) * x[3] * (np.sin(x[4]) * U[1] + np.cos(x[4]) * U[2]),
                    np.cos(x[4]) * x[3] * U[1] - np.sin(x[4]) * x[3] * U[2],
                    x[3] * np.sin(x[4]) / np.cos(x[5]) * U[1] + np.cos(x[4]) * x[3] * U[2] / np.cos(x[5])])
    return X

def draw(x,M):
    """
    Trace le motif.
    Entrée: x 
    Sortie: voir draw2() -> plot3D 
    """
    x = x.flatten()    
    ax.clear()
    ax.set_xlim3d(-20,20)
    ax.set_ylim3d(-20,20)
    ax.set_zlim3d(0,15)
    M = rotate(x[-3:], M)   # on prends les 3 dernieres composantes de x
    M = translate(x[:3], M)
    draw2(M)        
    ax.scatter(x[0],x[1],0,color='red')


fig = figure()
ax = Axes3D(fig)
x = np.transpose(array([[0,0,10,8,0.1,0.2,-10]])) #[x,y,z,v,phi,theta,psi]
u = np.transpose(array([[0,0,-0.1,0]])) #  u1, u2, u3, u4
n=100
dt = 0.05
# Motif
M = array([[0.0 ,0.0 ,10.0,0.0 ,0.0 ,10.0,0.0 ,0.0],
        [-1.0,1.0 ,0.0 ,-1.0,-0.2,0.0 ,0.2 ,1.0],
        [0.0 ,0.0 ,0.0 ,0.0 ,1.0 ,0.0 ,1.0 ,0.0],
        [1.0 ,1.0 ,1.0 ,1.0 ,1.0 ,1.0 ,1.0 ,1.0]])
# SIMULATION
draw(x,M)
for k in range(n):
    x=x+dt*f(x,u) # methode d'Euler
    draw(x,M)
    plt.pause(dt)