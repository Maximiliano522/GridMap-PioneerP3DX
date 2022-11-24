
from __future__ import print_function
import sim     
import time
import math
import sys 
import math as m
import numpy as np
import scipy.interpolate as spi
import trajectory as Traject
from skimage.draw import line
from astarmod import getTrajectoryA
import matplotlib.pyplot as plt
import os
import astarmod as ast
import cv2

#                """ Configuracion Inicial """

Kv = .2                                    # Constantes de aceleración
Kh = .5
                                            
END = 20                                  # TIEMPO
                                            
MAPSIZE = 50                               # Tamaño de Mapa inicial
                                            
SizeG = 0.1                                # Tamaño de celda

#                 """ Seleccion de trayectoria """

#xarr, yarr, xorg, yorg = Traject.Random()    # T = 300 - 600
#xarr, yarr, xorg, yorg = Traject.TZ()
#xarr, yarr = Traject.square()                # T = 80
#xarr, yarr = Traject.SQUARE()                # T = 350
#xarr, yarr = Traject.Diagonal()              # T = 50
#xorg, yorg = Traject.Diagonal()
#xarr, yarr = Traject.squareORIGIN()
#xorg, yorg = Traject.squareORIGIN()

# Imprimimos la trayectoria a seguir
#Traject.Grafica(xarr, yarr, xorg, yorg)

class Robot():

    def __init__(self):
    
        # Asignamos los handles a los motores
        err, self.motor_l = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
        err, self.motor_r = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
        err, self.robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)


        # Asignamos los handles a los sensores
        self.usensor = []
        for i in range(16):
            err, s = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), sim.simx_opmode_blocking)
            self.usensor.append(s)

        # Inicializamos los sensores
        for i in range(16):
           err, self.state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_streaming)

        # Inicialización de posición
        ret, self.carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)


        # if os.path.exists('Mapa.npz'):
        #     print('Mapa encontrado. Cargando...')
        #     MAPA = np.load('Mapa.npz')
        #     self.occgrid = MAPA['occgrid']                  # Cargamos la matriz con los valores de las celdas vacias
        #     self.tocc = MAPA['tocc']                        # Cargamos la matriz con los valores de las celdas ocupadas
        #     ConfigR = MAPA['Conf']

            
        #     self.IncX = int(ConfigR[0])                     # Cargamos el incremento en X para la transformación de coordenadas
        #     self.IncY = int(ConfigR[1])                     # Cargamos el incremento en y para la transformacion de coordenadas
        #     self.SizeGrid = ConfigR[2]                      # Cargamos el tamaño de rejilla preestablecido en el mapa
        #     self.MAPSIZE = int(ConfigR[3])                  # Cargamos el tamaño del mapa original
        
        
        # else:
        #     print('Creando nuevo Mapa...')
        #     self.occgrid = 0.5*np.ones((MAPSIZE,MAPSIZE))   # Creamos una matriz con valores de casilla sin explorar
        #     self.tocc = np.zeros((MAPSIZE,MAPSIZE))         # Creamos una matriz dónde guardaremos las celdas ocupadas
        #     self.SizeGrid = SizeG
        #     self.IncX = 0                                   # Establecemos los valores iniciales por default
        #     self.IncY = 0
        #     self.MAPSIZE = int(MAPSIZE)

        #     """""
        #     self.TimeTra = 0
        #     self.Explore1 = False
        #     self.Explore2 = False
        #     self.Explore3 = False
        #     self.Explore4 = False
        #     self.Explore5 = False
        #     self.Explore6 = False
        #     self.Explore7 = False
        #     self.Explore8 = False
        #     self.AjusteTiempo = 0

        #     """
        
        
        
        # self.CXo = int(self.MAPSIZE/2)                       # Centro original en x
        # self.CYo = int(self.MAPSIZE/2)                       # Centro original en Y
 
    def getDistanceReading(self, i):
        # Obtenemos la lectura del sensor
        err, self.state, Point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)

        if self.state == 1:
            # retornamos la magnitud del punto detectado
            return math.sqrt(sum(i**2 for i in Point))
        else:
            # Devuelve otro valor que sabemos que no puede ser verdadero
            return 199999

    def getSensorReading(self, i):
        # Obtenemos la lectura del sensor
        err, self.state, Point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)
        ret, srot = sim.simxGetObjectQuaternion(clientID, self.usensor[i], -1, sim.simx_opmode_blocking)

        return self.state, Point, srot, math.sqrt(sum(i**2 for i in Point)) #np.linalg.norm(Point)

    def Position (self):
        ret, self.carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        xw = self.carpos[0]
        yw = self.carpos[1]
        return xw, yw, self.carpos, carrot
    
    def Velocity(self, vLeft, vRight):
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, vLeft, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, vRight, sim.simx_opmode_blocking)

    def stop(self):
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, 0, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, 0, sim.simx_opmode_blocking)

    def angdiff(self, t1, t2):
        # The angle magnitude comes from the dot product of two vectors
        angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
        # The direction of rotation comes from the sign of the cross product of two vectors
        angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
        return m.copysign(angmag, angdir)

    def q2R(self, x,y,z,w):
        R = np.zeros((3,3))
        R[0,0] = 1-2*(y**2+z**2)
        R[0,1] = 2*(x*y-z*w)
        R[0,2] = 2*(x*z+y*w)
        R[1,0] = 2*(x*y+z*w)
        R[1,1] = 1-2*(x**2+z**2)
        R[1,2] = 2*(y*z-x*w)
        R[2,0] = 2*(x*z-y*w)
        R[2,1] = 2*(y*z+x*w)
        R[2,2] = 1/2*(x**2+y**2)
        return R

    def new_Map(grid_old,width,height,cords_copy):
        new_map = np.zeros((height,width)) 
        j_prime = cords_copy[0]
        for j in range(grid_old.shape[0]):
            i_prime = cords_copy[1]
            for i in range(grid_old.shape[1]):
                new_map[j_prime][i_prime] = grid_old[j][i]
                
                i_prime += 1
            j_prime += 1
        return new_map

    def interp(self, tiempo, pcix, pciy):


        """ Interpolador Pchip """"" 

        xnew = pcix(tiempo)
        ynew = pciy(tiempo) 
        

        """" Interpolador spline """ """""

        xc = spi.splrep(tarr, xarr, s=0)
        yc = spi.splrep(tarr, yarr, s=0)

        
        xnew = spi.splev(tiempo, xc, der=0)
        ynew = spi.splev(tiempo, yc, der=0)

        """""

        return {'x':xnew,'y':ynew}

    def Follow(self, CorrX, CorrY):
        
        tarr = np.linspace(0, 1000, CorrX.shape[0])
        ExpX = spi.PchipInterpolator(tarr, CorrX) 
        ExpY = spi.PchipInterpolator(tarr, CorrY)

        ts = time.time()

        x, y, self.carpos, carrot = self.Position() 

        tau = ts - t

        data = self.interp(tau, ExpX, ExpY)
        xd = data['x']
        yd = data['y']

        errp = m.sqrt((xd-self.carpos[0])**2 + (yd-self.carpos[1])**2)
        angd = m.atan2(yd-self.carpos[1], xd-self.carpos[0])
        errh = self.angdiff(carrot[2], angd)

        v = Kv*errp
        omega = Kh*errh

        ul = v/r - L*omega/(2*r)

        ur = v/r + L*omega/(2*r)
    
        """""
        if ul > 20:
            ul = ul - ((ul/100)*90)
        if ur > 20:
            ur = ur - ((ur/100)*90)
        if ul > 10:
            ul = ul - ((ul/100)*75)
        if ur > 10:
            ur = ur - ((ur/100)*75)
        if ul > 8:
            ul = ul - ((ul/100)*60)
        if ur > 8:
            ur = ur - ((ur/100)*60)
        if ul > 5:
            ul = ul - ((ul/100)*50)
        """

        xt.append(self.carpos[0])
        yt.append(self.carpos[1])

        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, ul, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, ur, sim.simx_opmode_blocking)

    def Trajectory(self):
        
        CorrX, CorrY = Traject.astra(self.path)
 
        #Traject.Grafica(CorrX, CorrY)
 
        self.Follow(CorrX,CorrY)
 
        """""
        if (time.time()-t) < 30 and self.Explore1 == False:
            CorrX, CorrY = Traject.Exp1()
            #Traject.Grafica(CorrX, CorrY)
            self.Follow(CorrX,CorrY)
            print('Estado 1')
        elif self.Explore1 == False: 
            self.Explore1 = True
            self.AjusteTiempo = self.AjusteTiempo + 70
        elif (time.time()-t) < 60 and self.Explore2 == False:
            CorrX, CorrY = Traject.ORG(self.carpos)
            #Traject.Grafica(CorrX, CorrY)
            self.Follow(CorrX,CorrY)
            print('Regresando')
        elif (time.time()-t) < 90 and self.Explore2 == False:
            CorrX, CorrY = Traject.Exp2()
            #Traject.Grafica(CorrX, CorrY)
            print('Estado 2')
            self.Follow(CorrX,CorrY)
        elif self.Explore2 == False:
            self.Explore2 = True
            self.AjusteTiempo = self.AjusteTiempo + 50
        elif (time.time()-t) < 120 and self.Explore3 == False:
            CorrX, CorrY = Traject.ORG(self.carpos)
            #Traject.Grafica(CorrX, CorrY)
            print('Regresando')
            self.Follow(CorrX,CorrY)
        elif (time.time()-t) < 150 and self.Explore3 == False:
            CorrX, CorrY = Traject.Exp3()
            #Traject.Grafica(CorrX, CorrY)
            self.Follow(CorrX,CorrY)
            print('Estado 3')
        elif self.Explore3 == False:
            self.Explore3 = True
        elif (time.time()-t) < 180 and self.Explore4 == False:
            CorrX, CorrY = Traject.ORG(self.carpos)
            #Traject.Grafica(CorrX, CorrY)
            self.Follow(CorrX,CorrY)
        elif (time.time()-t) < 210 and self.Explore4 == False:
            CorrX, CorrY = Traject.Exp4()
            #Traject.Grafica(CorrX, CorrY)
            self.Follow(CorrX,CorrY)
        """
        
    def Braitenberg (self):

        xw, yw, self.carpos, carrot = self.Position()

        for i in range (16):
            dist = self.getDistanceReading(i)
            if dist<noDetectionDist:
                if dist<maxDetectionDist:
                    dist=maxDetectionDist
                detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
            else:
                detect[i]=0
        
        vLeft=2
        vRight=2
        
        for i in range (16):
            vLeft=vLeft+braitenbergL[i]*detect[i]
            vRight=vRight+braitenbergR[i]*detect[i]
        
        xt.append(self.carpos[0])
        yt.append(self.carpos[1])

        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, vLeft, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, vRight, sim.simx_opmode_blocking)

    def LineS(self, xr, yr, row, col, i):

        self.state, point, srot, dist = self.getSensorReading(i)     
        x, y, spos, carrot = self.Position()
        R = self.q2R(srot[0], srot[1], srot[2], srot[3])
        spos = np.array(spos).reshape((3,1))
        #self.Dist.append(dist)
        #self.State.append(self.state)

        xt.append(spos[0])
        yt.append(spos[1])

        if self.state == True:

            opos = np.array(point).reshape((3,1))
                
            pobs = np.matmul(R, opos) + spos
            xs = pobs[0]
            ys = pobs[1]
            xo = self.CXo + m.ceil(xs/self.SizeGrid) + self.IncX
            yo = self.CXo - m.floor(ys/self.SizeGrid) + self.IncY

            if xo >= col:
                xo = xr
            elif xo <= 0:
                xo = xr
            if yo >= row:
                yo = yr
            elif yo <=0:
                yo = yr
                
            rows, cols = line(yr-1, xr-1, yo-1, xo-1)
            self.occgrid[rows, cols] = 0
            self.tocc[yo-1, xo-1] = 1

        else:

            opos = np.array([0,0,2.5]).reshape((3,1))

            pobs = np.matmul(R, opos) + spos
            xs = pobs[0]
            ys = pobs[1]
                
            xo = self.CXo + m.ceil(xs/self.SizeGrid) + self.IncX
            yo = self.CYo - m.floor(ys/self.SizeGrid) + self.IncY

            if xo >= col:
                xo = xr
            elif xo <= 0:
                xo = xr
            if yo >= row:
                yo = yr
            elif yo <=0:
                yo = yr

            rows, cols = line(yr-1, xr-1, yo-1, xo-1)  
            self.occgrid[rows, cols] = 0

    def Mapeo(self):
    
        xw, yw, self.carpos, carrot = self.Position()

        xr = self.CXo + m.ceil(xw/self.SizeGrid) + self.IncX           # Obtenemos la posicion en X de nuestro robot de acuerdo al marco global
        yr = self.CYo - m.floor(yw/self.SizeGrid)  + self.IncY         # Obtenemos la posicion en y de nuestro robot de acuerdo al marco global

        row, col = self.occgrid.shape                                  # Contamos el maximo de columnas y filas con el que disponemos

        if xr >= col:                                                  # Comprobamos si la posicion en X postiva se puede graficar en nuestra matriz
            print('Agregando columnas al final')                       # En caso de ser cierta la comprobacion, agregamos columnas al final de nuestras matrices 
            for i in range(self.MAPSIZE):
                self.occgrid = np.insert(self.occgrid, -1, .5, axis=1)
                self.tocc = np.insert(self.tocc, -1, 0, axis = 1)
        elif xr <= 0:                                                  # Comprobamos si la posicion en X es negativa y por lo tanto saldria de los limites de nuestra matriz
            print('Agregando columnas al inicio')                      # En caso de ser cierta la comprobacion, agregamos columnas al inicio de nuestras matrices 
            for i in range(self.MAPSIZE):
                self.occgrid = np.insert(self.occgrid, 0, .5, axis=1)
                self.tocc = np.insert(self.tocc, 0, 0, axis = 1)
            self.IncX = self.IncX + self.MAPSIZE                       # En este caso estaremos guardando el incremento en X para mantener nuestro marco global
            xr = self.CXo + m.ceil(xw/self.SizeGrid) + self.IncX       # Volvemos a obtener las cordenadaS en X en nuestro nuevo marco global ampliado
            
        if yr >= row:                                                  # Comprobamos si la posicion en Y postiva se puede graficar en nuestra matriz
            print('Insertando filas al final')                         # En caso de ser cierta la comprobacion, agregamos filas al final de nuestras matrices
            for i in range (self.MAPSIZE):
                self.occgrid = np.insert(self.occgrid, -1, .5, axis=0)
                self.tocc = np.insert(self.tocc, -1, 0, axis = 0)
        elif yr <= 0:                                                  # Comprobamos si la posicion en Y es negativa y por lo tanto saldria de los limites de nuestra matriz
            print('Insertando filas al inicio')
            for i in range(self.MAPSIZE):
                self.occgrid = np.insert(self.occgrid, 0, .5, axis=0)
                self.tocc = np.insert(self.tocc, 0, 0, axis = 0)
            self.IncY = self.IncY + self.MAPSIZE                       # En este caso estaremos guardando el incremento en Y para mantener nuestro marco global
            yr = self.CYo - m.floor(yw/self.SizeGrid) + self.IncY      # Volvemos a obtener las cordenadaS en X en nuestro nuevo marco global ampliado
            
        self.occgrid[yr-1, xr-1] = 0                                   # Mandamos los valores donde se encuentra el robot como celdas ocupadas

        row, col = self.occgrid.shape                                  # Volvemos a leer el tamaños de nuestras matrices por si hubo cambios y enviarlos a nuestra funcion Line Sensors

        self.Dist = []
        self.State = []

        xt.append(self.carpos[0])
        yt.append(self.carpos[1])

        for i in range(16):                                            # En un rango de 15 sensores graficamos lo que detecta cada uno de ellos
            self.LineS(xr,yr,row,col, i)

        """""
        ul = 2.0
        ur = 2.0

        for i in range(16):
            if self.State[i]:
                dist = self.getDistanceReading(i)
                if dist<noDetectionDist:
                    if dist<maxDetectionDist:
                        dist=maxDetectionDist
                    detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
                else:
                    detect[i]=0
                    
                ul=ul+braitenbergL[i]*detect[i]
                ur=ur+braitenbergR[i]*detect[i]

        self.Velocity(ul,ur)
        """""
    def GetMap_Trayectory(self):
        # if os.path.exists('Mapa-espiral.txt'):
        #     print('Map found. Loading...')
        #     occgrid = np.loadtxt('Mapa-espiral.txt')
        #     tocc = 1.0*(occgrid > 0.5)
        #     occgrid[occgrid > 0.5] = 0
        #     maze = tocc    
        #     tam = np.shape(occgrid)
        #     self.CXo = int(tam[0]/2)
        #     self.CYo = int(tam[0]/2)
        #     self.SizeGrid = SizeG
        #     self.IncX = 0
        #     self.IncY = 0
        
        if os.path.exists('Mapa.npz'):
            print('Mapa encontrado. Cargando...')
            MAPA = np.load('Mapa.npz')
            self.occgrid = MAPA['occgrid']                  # Cargamos la matriz con los valores de las celdas vacias
            self.tocc = MAPA['tocc']                        # Cargamos la matriz con los valores de las celdas ocupadas
            ConfigR = MAPA['Conf']

            
            self.IncX = int(ConfigR[0])                     # Cargamos el incremento en X para la transformación de coordenadas
            self.IncY = int(ConfigR[1])                     # Cargamos el incremento en y para la transformacion de coordenadas
            self.SizeGrid = ConfigR[2]                      # Cargamos el tamaño de rejilla preestablecido en el mapa
            self.MAPSIZE = int(ConfigR[3])            
            maze = self.tocc
            
        plt.imshow(maze, cmap='gray')
        plt.show()
        
        kernel = np.ones((8,8),np.uint8)
        
        mazed = cv2.dilate(maze,kernel,iterations = 1)
        
        plt.imshow(mazed, cmap='gray')
        plt.show()
        
        xw, yw, self.carpos, carrot = self.Position()

        xr = self.CXo + m.ceil(xw/self.SizeGrid) + self.IncX           # Obtenemos la posicion en X de nuestro robot de acuerdo al marco global
        yr = self.CYo - m.floor(yw/self.SizeGrid)  + self.IncY         # Obtenemos la posicion en y de nuestro robot de acuerdo al marco global
        
        start = (xr,yr)
        end = (120,61)
        
        self.path = ast.astar(mazed.tolist(), start, end, allow_diagonal_movement=True)
        print(self.path)
        

        
    def Finaly(self):
        plt.imshow(self.tocc+self.occgrid, cmap='gray')
        plt.show()
        Config = [self.IncX,self.IncY,self.SizeGrid,self.MAPSIZE]                   # Definimos nuestra matriz con las configuraciones a guardar
        np.savez('Mapa', tocc = self.tocc, occgrid = self.occgrid, Conf = Config)   # Guardamos en un archivo ".npz" nuestras matrices

print ('Programa Iniciado')
sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',-1,True,True,5000,5)  # Conexión a CoppeliaSim

if clientID!=-1:
    print ('Conectados al remote API')
    
    robot = Robot() 

    r = 0.5*0.195
    L = 0.311

    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = np.zeros(16)
    braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    
    
    
    xt = []
    yt = []
    

    ttime =  END


    t = time.time()
    robot.GetMap_Trayectory()
    #                                        """ Ciclo de trabajo """

    while (time.time()-t) < END:
        # for i in range (8):
        #     while robot.getDistanceReading(i) <= 1:       # Comprobamos si algun sensor detecta un objeto
        #         robot.Braitenberg() 
        #         robot.Mapeo()                             # Usamos la funcion Braitenberg para evadir objetos
        #         print ("Evadiendo obstaculo")
        robot.Trajectory()
        #robot.Velocity(3,3)
        #robot.Mapeo()                                     # El robot seguira la trayectoria hasta encontrar un objeto
        print ("Siguiendo Trayectoria") 
                            
    robot.stop()                                          # Detenemos nuestro robot
    Traject.GrafOut(xt,yt)                                # Imprimimos resultados
    #robot.Finaly()

    
    sim.simxGetPingTime(clientID)                        # Desconectamos del Remote Api para finalizar el programa
    sim.simxFinish(clientID)
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
else:
    print ('Conexion fallida a remote API server')
print ('Fin del programa')
sys.exit('Sin conexion')











