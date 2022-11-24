
import matplotlib.pyplot as mpl   
import numpy as np
import scipy.interpolate as spi
import math as mt

TamExp = 30
TamEspiral = 30  # 900 m2

def INTERPOL(CorrX, CorrY):

    tarr = np.linspace(0, 100, CorrX.shape[0])
    tnew = np.linspace(0, 100, 500)

    """ Interpolador Pchip """"" 

    pcix = spi.PchipInterpolator(tarr, CorrX) 
    pciy = spi.PchipInterpolator(tarr, CorrY)

    xnew = pcix(tnew)
    ynew = pciy(tnew) 

    return xnew, ynew

def path2cells(path):
    rows = np.array([])
    cols = np.array([])
    for p in path:
        rows = np.append(rows, p[0])
        cols = np.append(cols, p[1])
    return rows, cols

def astra (path):
    CorrX, CorrY = path2cells(path)
    
    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def spiral():
    # Center of the map
    cx = 0
    cy = 0
    
    xlist = []
    ylist = []
    
    # First point is the center
    xlist.append(cx)
    ylist.append(cy)
            
    # Spiral
    for i in range(250):
        vt = i / 40 * mt.pi
        y = (vt * 3 + 3) * mt.sin(vt)
        x = (vt * 3 + 3) * mt.cos(vt)                
    
    
        # Update last point
        xlist.append(x)
        ylist.append(y)
    
    xarr = np.array(xlist)
    yarr = np.array(ylist)
    cx = np.array(cx)
    cy = np.array(cy)

    return xarr,yarr,cx,cy

def Espiral():

    CorrX = np.array([0])
    CorrY = np.array([0])

    aux = -1
    for i in range (TamEspiral):
        for i in range (4):
            CorrX = np.append(CorrX,aux)
            if i ==1:
                aux = aux * - 1
        aux = (aux +1) * - 1
    CorrX = np.insert(CorrX, -1,101)
     
    aux = -1
    for i in range (TamEspiral):
        for i in range (4):
            CorrY = np.append(CorrY,aux)
            if i ==1:
                aux = aux * - 1
        aux = (aux +1) * - 1
    CorrY = np.insert(CorrY, 0,0)
    #print(CorrX)
    #print(CorrY)

    xnew, ynew = INTERPOL(CorrX, CorrY)

    return xnew, ynew

def ORG(carpos):

    CorrX = np.array([carpos[0],0])
    CorrY = np.array([carpos[1],0])

    xnew, ynew = INTERPOL(CorrX, CorrY)

    return xnew, ynew

def Exp1():

    CorrX = np.array([0,TamExp])
    CorrY = np.array([0,TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp2():

    CorrX = np.array([0,0])
    CorrY = np.array([0,TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp3():

    CorrX = np.array([0,-TamExp])
    CorrY = np.array([0,TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp4():

    CorrX = np.array([0,-TamExp])
    CorrY = np.array([0,0])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp5():

    CorrX = np.array([0,-TamExp])
    CorrY = np.array([0,-TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp6():

    CorrX = np.array([0,0])
    CorrY = np.array([0,-TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp7():

    CorrX = np.array([0,TamExp])
    CorrY = np.array([0,-TamExp])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def Exp8():

    CorrX = np.array([0,TamExp])
    CorrY = np.array([0,0])

    xnew, ynew = INTERPOL(CorrX, CorrY)
    
    return xnew, ynew

def TZ(rows, cols):

    rows = int(((rows/2) * 0.1)) - 4
    cols = int(((cols/2) * 0.1)) - 4
    
    xarr = np.array([-cols, cols])
    yarr = np.array([-rows, rows])

    for i in range (5):
        xarr = np.insert(xarr, -1, -cols)
        xarr = np.insert(xarr,-1, cols)
    
    aux = - rows

    for i in range (10):
        aux = aux + int(rows/5)
        yarr = np.insert(yarr, -1, aux)

    xarrOrig = xarr
    yarrOrig = yarr

    #yarr = np.sort(yarr)
    #xarr = np.sort(xarr)

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

def Random():

#TrayectoriaRandom  
    
    xarr = np.random.randint(-6, 6, 10)
    yarr = np.random.randint(-6, 6, 10)

    
    xarr = np.insert(xarr, (0), [-5])
    yarr = np.insert(yarr, (0), [-6])

    xarrOrig = xarr
    yarrOrig = yarr

    #yarr = np.sort(yarr)
    #xarr = np.sort(xarr)

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
    
    xarr = np.array([   -2.5,  -1.5,   -.5,  .5,  1.5,  2.5,   3.5,  4, 5])
    yarr = np.array([   -2.5,  -1.5,   -.5,  .5,  1.5,  2.5,   3.5,  4, 5])
    return xarr, yarr

def Grafica1(xarr, yarr, xorg, yorg):

    mpl.scatter(xarr,yarr, label ='Trayectoria')
    mpl.scatter(xorg, yorg, label = 'Puntos random')
    mpl.scatter(0,0, label = 'Centro', color = 'Red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA A SEGUIR')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc='upper center', bbox_to_anchor=(0.4, -0.05), fancybox=True, shadow=True, ncol=4)
    mpl.show()

def Grafica(xarr, yarr):

    mpl.scatter(xarr,yarr, label ='Trayectoria')
    mpl.scatter(0,0, label = 'Centro', color = 'Red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA A SEGUIR')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc='upper center', bbox_to_anchor=(0.4, -0.05), fancybox=True, shadow=True, ncol=4)
    mpl.show()

def GrafOut1(xarr, yarr, xorg, yorg):
    
    mpl.scatter(xarr,yarr, label ='Trayectoria seguida por el robot')
    mpl.scatter(xorg, yorg, label = 'Puntos random')
    mpl.scatter(0,0, label = 'Centro', color = 'red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA CONSEGUIDA')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc= 'upper center', bbox_to_anchor=(.4, -0.05), fancybox=True, shadow=True, ncol=4, fontsize = 'x-small')
    mpl.show()

def GrafOut(xarr, yarr):
    
    mpl.scatter(xarr,yarr, label ='Trayectoria seguida por el robot')
    mpl.scatter(0,0, label = 'Centro', color = 'red')
    mpl.scatter(xarr[0],yarr[0], label = 'Punto inicial', color = 'black')
    mpl.title('TRAYECTORIA CONSEGUIDA')
    mpl.xlabel('X', loc = 'right')
    mpl.ylabel('Y', loc = 'top')
    mpl.legend(loc= 'upper center', bbox_to_anchor=(.4, -0.05), fancybox=True, shadow=True, ncol=4, fontsize = 'x-small')
    mpl.show()


