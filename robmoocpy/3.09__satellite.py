from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

s=2
ax=init_figure(-s,s,-s,s)
clear(ax)
draw_disk(array([[0],[0]]),0.2,ax,"blue",1,1)

x=array([[1.22],[0],[0],[1]])
draw_disk(array([[x[0]],[x[1]]]),0.1,ax,"red",1)
pause(1)    





 
