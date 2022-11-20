import numpy as np
import matplotlib.pyplot as plt
import astarmod 

map = np.load('Mapa.npz') 

occgrid = map['occgrid']
tocc = map['tocc']
mapa_c = tocc + occgrid



plt.imshow(tocc+occgrid, cmap='gray')
plt.show()


start = (0, 0)
end = (1, 2)

print(mapa_c)
path =  astarmod.astar(mapa_c, start, end, allow_diagonal_movement=True) 


print(path)





