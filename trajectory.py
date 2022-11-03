
import matplotlib.pyplot as mpl   
import numpy as np
import scipy.interpolate as spi

def Random():

#TrayectoriaRandom  
    
    xarr = np.random.randint(-6, 6, 10)
    yarr = np.random.randint(-6, 6, 10)

    xarr = np.insert(xarr, (0), [-5])
    yarr = np.insert(yarr, (0), [-6])

    xarrOrig = xarr
    yarrOrig = yarr

    tarr = np.linspace(0, 11, xarr.shape[0])
    tnew = np.linspace(0, 11, 500)

    """ Interpolador Pchip """"" 

    pcix = spi.PchipInterpolator(tarr, xarr) 
    pciy = spi.PchipInterpolator(tarr, yarr)

    xnew = pcix(tnew)
    ynew = pciy(tnew) 

    """""  Intrpolador Spline """"" """""

    xc = spi.splrep(tarr, xarr, s=0)
    yc = spi.splrep(tarr, yarr, s=0)

    xnew = spi.splev(tnew, xc, der=0)
    ynew = spi.splev(tnew, yc, der=0)

    """""
    return xnew, ynew, xarrOrig, yarrOrig

def squareORIGIN():

    # Trayectoria de Cuadrado Pequeño origen - Tiempo = 80

    xarr = np.array([0, 0.5,1,2.5,  3,  3,  3, 3, 2.5 ,0.5,0, 0,0, 0,0])
    yarr = np.array([0, 0,0,0,      0,  0.5, 2.5, 3, 3, 3,3,2.5,1,0.5,0])
    return xarr, yarr

def square():

    # Trayectoria de Cuadrado Pequeño - Tiempo = 80
    
    xarr = np.array([  -4,  -3.5,  -2.5,  -2,    -2,     -2,  -2,  -2.5,  -3.5,  -4,    -4,    -4,   -4])
    yarr = np.array([  -4,    -4,    -4,  -4,  -3.5,   -2.5,  -2,    -2,    -2,  -2,  -2.5,  -3.5,   -4])
    return xarr, yarr

def SQUARE():

    # Trayectoria  Cuadrado Grande - Tiempo = 350 

    xarr = np.array([  -4,  -3.5,    -1,     1,   3.5,     4,     4,   4,   4,    4,   2.5,   -1,   -3,    -4,  -4, -4,  -4,  -4,    -4])
    yarr = np.array([-5.5,  -5.5,  -5.5,  -5.5,  -5.5,  -5.5,  -4.5,  -2,   2,  5.5,   5.5,  5.5,  5.5,   5.5,   4,  1,  -1,  -4,  -5.5])
    return xarr, yarr

def Diagonal():

    # Trayectoria Diagonal - Tiempo = 50
    
    xarr = np.array([  -4,  -3.5,  -2.5,  -1.5,   -.5,  .5,  1.5,  2.5,   3.5,  4, 5])
    yarr = np.array([  -4,  -3.5,  -2.5,  -1.5,   -.5,  .5,  1.5,  2.5,   3.5,  4, 5])
    return xarr, yarr

def Grafica(xarr, yarr, xorg, yorg):

    mpl.scatter(xarr,yarr, label ='Trayectoria')
    mpl.scatter(xorg, yorg, label = 'Puntos random')
    mpl.scatter(0,0, label = 'Centro', color = 'Red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA A SEGUIR')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc='upper center', bbox_to_anchor=(0.4, -0.05), fancybox=True, shadow=True, ncol=4)
    mpl.show()

def GrafOut(xarr, yarr, xorg, yorg):
    
    mpl.scatter(xarr,yarr, label ='Trayectoria seguida por el robot')
    mpl.scatter(xorg, yorg, label = 'Puntos random')
    mpl.scatter(0,0, label = 'Centro', color = 'red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA CONSEGUIDA')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc= 'upper center', bbox_to_anchor=(.4, -0.05), fancybox=True, shadow=True, ncol=4, fontsize = 'x-small')
    mpl.show()


