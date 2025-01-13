import rospy 
import time 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ModuloAnalisis import *
from ModuloMovimiento import *


def CallbackScan(msg):
    """Funcion de callback para el topic de scan"""
    global DatosScan, ScanLeido
    
    #Guardamos los datos de scan en una variable global
    ScanLeido = True
    DatosScan = msg
    
def CallbackOdom(msg):
    """Funcion de callback para el topic de odometria"""
    global PosicionActual, OdomLeido
    
    #Guardamos los datos de odometria en una variable global
    OdomLeido = True
    PosicionActual = msg.pose.pose
    

def PublishMotores(Vlin, Vang):
    """Funcion para publicar velocidades en los motores"""
    global PubMotores
    
    #Creamos el mensaje de tipo Twist
    msg = Twist()
    msg.linear.x = Vlin
    msg.angular.z = Vang
    
    #Publicamos el mensaje
    PubMotores.publish(msg)

#TODO: Definicion de variables globales

#Creamos el nodo principal
rospy.init_node('Main')
rospy.loginfo('Nodo principal creado')

#Creamos el subcriptor de scan 
DatosScan = None
ScanLeido = False

rospy.Subscriber('/scan', LaserScan, CallbackScan)
rospy.loginfo('Subscriptor de scan creado')
rospy.loginfo('Esperando datos de scan')
while not ScanLeido:
    time.sleep(0.5)
rospy.loginfo('Datos de scan leidos')

#Creamos el subcriptor de odometria
PosicionActual = None
OdomLeido = False
rospy.Subscriber('/odom', Odometry, CallbackOdom)
rospy.loginfo('Subscriptor de odometria creado')
rospy.loginfo('Esperando datos de odometria')
while not OdomLeido:
    time.sleep(0.5)
rospy.loginfo('Datos de odometria leidos')

#Creamos el publicados para los motores
PubMotores = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.loginfo('Publicador de motores creado')

#Creamos el algoritmo split and merge
sm = SplitAndMerge(dist=0.09, angle=0.0, purge_pts = 8, purge_len = 0.3) 


i=100
Rate = rospy.Rate(10)

#Variables 
Estado = 0 #0: Seguir, 1: Pared enfrente, 2: deja de detectar parede derecha
print('Esperando datos de scan')
while not rospy.is_shutdown():

    #Obtenemos la lista de coordenadas polares
    ListaPolar = DatosScan.ranges
    
    #Convertimos las coordenadas polares a cartesianas
    ListaCart = Polar2Cart(ListaPolar)
    
    #Obtenemos los segmentos de la nube de puntos
    ListaSegmentos = sm(ListaCart)
    if ListaSegmentos is None:
        continue
    
    else:
        #print('Segmentos:', len(ListaSegmentos))
        
        #Elegimos el segmento a seguir
        SegSeguir,DistanciaY = elegir_seguir(ListaSegmentos)
        SegEnfrente, DistanciaX =  EleguirEnfrente(ListaSegmentos)
            
        #Plotemos los segmentos si hay algo que seguir o enfrente
        if SegSeguir is not None and SegEnfrente is not None:
            sm.plot(ListaCart, [SegSeguir, SegEnfrente])
        
        elif SegSeguir is not None:
            sm.plot(ListaCart, [SegSeguir])
        
        elif SegEnfrente is not None:
            sm.plot(ListaCart, [SegEnfrente])
            
        #Establecemos el estado del robot
        if SegSeguir is not None and (DistanciaX is None or DistanciaX > 1):
            #Si hay un segmento a seguir y no hay pared enfrente
            Estado = 0
        
        elif SegSeguir is not None and DistanciaX <= 1:
            #Si hay un segmento a seguir y hay pared enfrente
            Estado = 1
        elif SegEnfrente is not None and Estado == 2:
            #Si hay pared enfrente y no hay segmento a seguir 
            Estado = 1
        
        elif SegSeguir is None and Estado == 0:
            #Si no hay segmento a seguir y no hay pared enfrente
            Estado = 2
            i = 0 
            
        #Mantenemos el estado 2 por 7 iteracciones            
        if i <=7: 
            Estado= 2
        

        #Calculamos el angulo con el segmento
        if SegSeguir is not None:
            #Obtenemo las coordenadas de los puntos
            x0, y0 = SegSeguir.p0
            x1, y1 = SegSeguir.p1
            
            Angulo = np.arctan((y1 - y0)/(x1 - x0))
        else:
            Angulo = 0
    
        #Valores de iteraccion 
        print('Estado:', Estado)
        print('\tDistanciaX:', DistanciaX, '\n\tDistanciaY:', DistanciaY, '\n\tAngulo:', Angulo)
        
        #Calculamos las velocidades
        Vlin, Vang = CalcularVelocidades(Estado, DistanciaX, DistanciaY, Angulo)
        
        #Publicamos las velocidades    
        print('Vlin:', Vlin, 'Vang:', Vang)
    #Publicamos las velocidades
    PublishMotores(Vlin, Vang)
    i+=1
    
    #Esperamos un tiempo
    Rate.sleep()
    print("************************************")
