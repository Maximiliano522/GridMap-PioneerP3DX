
from __future__ import print_function
from tkinter import Y
import sim     
import time
import math
import sys 
import math as m
import numpy as np
import scipy.interpolate as spi
import trajectory as Traject
from skimage.draw import line
import matplotlib.pyplot as plt
import os

Kv = 0.2 # 1    0.3
Kh = 0.3 # 2.5  0.5

#Tiempo
END = 200

MAPZISE = 100

""" Seleccion de trayectoria """
#xarr, yarr, xorg, yorg = Traject.Random()     # T = 300 - 600
#xarr, yarr = Traject.square()                # T = 80
#xarr, yarr = Traject.SQUARE()                # T = 350
#xarr, yarr = Traject.Diagonal()              # T = 50


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
           err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_streaming)

        # Inicialización de posición
        ret, carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)

        if os.path.exists('map.txt'):
            print('Map found. Loading...')
            self.occgrid = np.loadtxt('map.txt')
            self.tocc = 1.0*(self.occgrid > 0.5)
            self.occgrid[self.occgrid > 0.5] = 0
        else:
            print('Creating new map')
        # x, y = Robot.position()
            self.occgrid = 0.5*np.ones((MAPZISE,MAPZISE))
            self.tocc = np.zeros((MAPZISE,MAPZISE))

        self.CXo = int(MAPZISE/2)
        self.CYo = int(MAPZISE/2)
        
        self.IncX = 0
        self.IterIX = 0
        self.IncY = 0
        self.IterIY = 0

       

    def getDistanceReading(self, i):
        # Obtenemos la lectura del sensor
        err, State, Point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)

        if State == 1:
            # retornamos la magnitud del punto detectado
            return math.sqrt(sum(i**2 for i in Point))
        else:
            # Devuelve otro valor que sabemos que no puede ser verdadero
            return 199999

    def getSensorReading(self, i):
        # Obtenemos la lectura del sensor
        err, State, Point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)

        return err, State, np.linalg.norm(Point), detectedObj, detectedSurfNormVec

    def PositionTrayectory (self):
        ret, carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        return carpos, carrot, ret

    def Position (self):
        ret, carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        xw = carpos[0]
        yw = carpos[1]
        return xw, yw

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

    def world_to_grid(self, point):
        grid_center_coordinates = (MAPZISE/2, MAPZISE/2)
        scaling_factor = MAPZISE/2
        delta_x = (point[0]/15.0) * scaling_factor
        delta_y = (point[1]/15.0) * scaling_factor
        grid_coordinates = None
        grid_coordinates = (int(round(grid_center_coordinates[0]+delta_x)), int(round(grid_center_coordinates[1]-delta_y)))
        return grid_coordinates

    def interp(self, tiempo):


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

    def Trajectory(self):

        ts = time.time()

        carpos, carrot = self.PositionTrayectory() 

        tau = ts - t

        data = self.interp(tau)
        xd = data['x']
        yd = data['y']

        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = self.angdiff(carrot[2], angd)

        v = Kv*errp
        omega = Kh*errh

        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)

        xt.append(carpos[0])
        yt.append(carpos[1])


        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, ul, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, ur, sim.simx_opmode_blocking)
        
    def Braitenberg (self):
        
        carpos = self.Position() 

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
        
        xt.append(carpos[0])
        yt.append(carpos[1])

        return vLeft, vRight

    def Mapeo(self):
        
        iter = 0
        xw, yw = self.Position()

        xr = self.CXo + m.ceil(xw/0.1) + self.IncX
        yr = self.CYo - m.floor(yw/0.1)  + self.IncY

        row, col = self.occgrid.shape       

        if xr >= col:
            print('Agregando columnas al final')
            for i in range(MAPZISE):
                self.occgrid = np.insert(self.occgrid, -1, .5, axis=1)
                self.tocc = np.insert(self.tocc, -1, 0, axis = 1)
            xw, yw = self.Position()
            xr = self.CXo + m.ceil(xw/0.1) + self.IncX
            yr = self.CYo - m.floor(yw/0.1) + self.IncY
        elif xr <= 0:
            print('Agregando columnas al inicio')
            for i in range(MAPZISE):
                self.occgrid = np.insert(self.occgrid, 0, .5, axis=1)
                self.tocc = np.insert(self.tocc, 0, 0, axis = 1)
            self.IterIX = self.IterIX + 1
            self.IncX = self.IterIX * MAPZISE
            xw, yw = self.Position()
            xr = self.CXo + m.ceil(xw/0.1) + self.IncX
            yr = self.CYo - m.floor(yw/0.1) + self.IncY
            
        if yr >= row:
            print('Insertando filas al final')
            for i in range(MAPZISE):
                self.occgrid = np.insert(self.occgrid, -1, .5, axis=0)
                self.tocc = np.insert(self.tocc, -1, 0, axis = 0)
            xw, yw = self.Position()
            xr = self.CXo + m.ceil(xw/0.1) + self.IncX
            yr = self.CYo - m.floor(yw/0.1) + self.IncY
            
            #print("\n",xr, "\n", yr)
        elif yr <= 0:
            print('Insertando filas al inicio')
            for i in range(MAPZISE):
                self.occgrid = np.insert(self.occgrid, 0, .5, axis=0)
                self.tocc = np.insert(self.tocc, 0, 0, axis = 0)
            self.IterIY = self.IterIY + 1
            self.IncY = self.IterIY * MAPZISE
            xw, yw = self.Position()
            xr = self.CXo + m.ceil(xw/0.1) + self.IncX
            yr = self.CYo - m.floor(yw/0.1) + self.IncY
            
        row, col = self.occgrid.shape
        
        self.occgrid[yr-1, xr-1] = 0

        uread = []
        ustate = []
        upt = []

        

        for i in range(0,16):
            err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)
            ret, objpos = sim.simxGetObjectPosition(clientID, detectedObj, -1, sim.simx_opmode_blocking)
            uread.append(np.linalg.norm(point))
            upt.append(point)
            ustate.append(state)
            ret, srot = sim.simxGetObjectQuaternion(clientID, self.usensor[i], -1, sim.simx_opmode_blocking)
            ret, spos = sim.simxGetObjectPosition(clientID, self.usensor[i], -1, sim.simx_opmode_blocking)
            R = self.q2R(srot[0], srot[1], srot[2], srot[3])
            spos = np.array(spos).reshape((3,1))

            iter = iter +1

            if state == True:

                opos = np.array(point).reshape((3,1))
                
                pobs = np.matmul(R, opos) + spos
                xs = pobs[0]
                ys = pobs[1]
                xo = self.CXo + m.ceil(xs/0.1) + self.IncX
                yo = self.CXo - m.floor(ys/0.1) + self.IncY
                if xo >= col:
                    xo = xr
                    yo = yr
                elif xo <= 0:
                    xo = xr
                    yo = yr 
                if yo >= row:
                    yo = yr
                    xo = xr
                elif yo <=0:
                    xo = xr
                    yo = yr
                
                rows3, cols3 = line(yr-1, xr-1, yo-1, xo-1)
                #cols3 = cols3 - iter 
                #rows3 = rows3 - iter
                self.occgrid[rows3, cols3] = 0
                self.tocc[yo-1, xo-1] = 1

            else:

                opos = np.array([0,0,1]).reshape((3,1))

                pobs = np.matmul(R, opos) + spos
                xs = pobs[0]
                ys = pobs[1]
                
                xo = self.CXo + m.ceil(xs/0.1) + self.IncX
                yo = self.CYo - m.floor(ys/0.1) + self.IncY

                if xo >= col:
                    xo = xr
                    yo = yr
                if xo <= 0:
                    xo = xr
                    yo = yr 
                if yo >= row:
                    yo = yr
                    xo = xr
                if yo <=0:
                    xo = xr
                    yo = yr

                rows3, cols3 = line(yr-1, xr-1, yo-1, xo-1)
            
                #cols3 = cols3 - iter 
                
                #rows3 = rows3 - iter
                
                self.occgrid[rows3, cols3] = 0
                
        ul = 2.0
        ur = 2.0

        for i in range (16):
            if (self.getDistanceReading(i) <= 2.5):
                ul, ur = self.Braitenberg()


        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, ul, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, ur, sim.simx_opmode_blocking)

    def Finaly(self):
        plt.imshow(self.tocc+self.occgrid, cmap='gray')
        plt.show()
        #np.savetxt('map.txt', self.tocc+self.occgrid)

    

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
    

    #                                        """ Ciclo de trabajo"""""

    while (time.time()-t) < END:

         
        robot.Mapeo()                              
        
                            
    robot.stop()                                         # Detenemos nuestro robot
    #Traject.GrafOut(xt,yt,xorg,yorg)                     # Imprimimos resultados
    robot.Finaly()

    
    sim.simxGetPingTime(clientID)                        # Desconectamos del Remote Api para finalizar el programa
    sim.simxFinish(clientID)
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
else:
    print ('Conexion fallida a remote API server')
print ('Fin del programa')
sys.exit('Sin conexion')











