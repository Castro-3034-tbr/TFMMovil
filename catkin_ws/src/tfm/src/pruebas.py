import rospy
import time
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

from ModuloMovimiento import *
from ModuloAnalisis import *

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

def elegir_seguir(segmentos):
    """Funcion para elegir el segmento a seguir"""
    #Partimos todos los segmentos en dos y nos quedamos con la parte positiva
    DistanciaMin=None
    SegmentoMin=None
    for seg in segmentos:
        
        
        #Comprobamos que el segmentos no es vertical
        if abs(seg.p0[0] - seg.p1[0]) > 0.07:
            #A partir de la ecuacion de la recta y = mx + b
            #Calculamos el valor de y para x = 0
            y= seg.ecuacion[0]*0 + seg.ecuacion[1]
            
            if y < 0 and (seg.p0[0] > 0 or seg.p1[0] > 0):
                DistanciaY = np.abs(y)
                if DistanciaMin is None or DistanciaY < DistanciaMin:
                    DistanciaMin = DistanciaY
                    SegmentoMin = seg
    
    return SegmentoMin, DistanciaMin

def EleguirEnfrente(segmentos):
    
    DistanciaMin=None
    SegmentoMin=None
    for seg in segmentos:
        
        
        #Comprobamos que el segmentos no es vertical
        if abs(seg.p0[0] - seg.p1[0]) < 0.7: 
            #A partir de la ecuacion de la recta y = mx + b
            #Calculamos el valor de y para x = 0
            
            x = (0 - seg.ecuacion[1])/seg.ecuacion[0]
            
            #Calculamos la distancia en x 
            if x > 0 and (-0.5<seg.p0[1]  or -0.5<seg.p1[1]):
                DistanciaX = np.sqrt(seg.p0[0]**2 + 0**2)
                
                if DistanciaMin is None or DistanciaX < DistanciaMin:
                    DistanciaMin = DistanciaX
                    SegmentoMin = seg
                
            
    
    return SegmentoMin, DistanciaMin


def CalcularVelocidades(Estado, DistanciaX = None, DistanciaY = None , Angulo = None, DistanciaObjetivo = 0.5):
    """Funcion para calcular las velocidades en funcion de la distancia"""
    
    global pid, DistanciaObjetivoY, AnguloObjetivo 
    
    #Definimos las variables de velocidad
    Vlin = 0.0
    Vang = 0.0
    
    #Estado 0: Seguir Pared 
    if Estado == 0:
        #Velocidad lineal constante
        Vlin = 0.25
        
        #Calculo de los errores 
        errorY = DistanciaObjetivoY - DistanciaY
        errorAngulo = AnguloObjetivo - Angulo
        
        #Velocidad angular proporcional al error en Y
        Vang = pid.compute(errorY, errorAngulo, 0.1)

    #Estado 1: Esquina con pared enfrente
    elif Estado == 1:
        #Velocidad lineal proporcional al error en X
        Vlin= 0
        
        kv = 0.2
        Vlin=kv*DistanciaX
        
        #Velocidad angular constante
        Vang = 0.4
    
    #Estado 2: Deja de detectar pared derecha
    elif Estado == 2:
        #Velocidad lineal constante
        Vlin = 0.2
        #Velocidad angular constante
        Vang = -0.50
    
    return Vlin, Vang



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


#Definimos un controlador PID 
#pid = PIDController(0.6, 0.2, 0.15)
pid = PDController(0.35, 0.4, 0.1, 0.1, 0.6) #kpDistancia, kdDistancia, kpAngulo, kdAngulo , Ponderacion = 0.5

#Valores objetivo
DistanciaObjetivoY = 0.5
DistaciaObjetivoX = 1
AnguloObjetivo = 0.0

Rate = rospy.Rate(10)

#Variables 
Estado = 0 #0: Seguir, 1: Pared enfrente, 2: deja de detectar parede derecha, 3:Non detecta nada 
i=100
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
        
        #print('Seguir:', SegSeguir is None , 'Enfrente:', SegEnfrente is None)
        
        #FIXME: Eleccion del estado 
        print("Seguir:", SegSeguir, "Enfrente:", SegEnfrente)
        if SegSeguir is not None and SegEnfrente is not None:
            sm.plot(ListaCart, [SegSeguir, SegEnfrente])
        
        elif SegSeguir is not None:
            sm.plot(ListaCart, [SegSeguir])
        
        elif SegEnfrente is not None:
            sm.plot(ListaCart, [SegEnfrente])
        
        if SegSeguir is not None and (DistanciaX is None or DistanciaX > 1):
            Estado = 0
        
        elif SegSeguir is not None and DistanciaX <= 1:
            Estado = 1
        elif SegEnfrente is not None and Estado == 2:
            Estado = 1
        
        elif SegSeguir is None and Estado == 0:
            Estado = 2
            i = 0 
        
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
        
        
        
        print('Vlin:', Vlin, 'Vang:', Vang)
    
    #Publicamos las velocidades
    PublishMotores(Vlin, Vang)
    
    
    i+=1
    
    Rate.sleep()
    print("************************************")