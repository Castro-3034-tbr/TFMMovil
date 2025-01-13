import numpy as np
import sys
import queue
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA



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
    #Retornamos el segmento que esta mas cerca y la distancia
    return SegmentoMin, DistanciaMin

def EleguirEnfrente(segmentos):
    """Funcion para elegir el segmento que esta enfrente"""
    DistanciaMin=None
    SegmentoMin=None
    for seg in segmentos:
        #Comprobamos que el segmentos  es vertical
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
    #Retornamos el segmento que esta mas cerca y la distancia          
    return SegmentoMin, DistanciaMin

def Polar2Cart(ListaPolar):
    """Convierte una lista de coordenadas polares a cartesianas"""
    
    global DeltaAng, RangoMax, RangoMin
    
    #Definimos las variables locales
    ListaCart = []
    
    #Lista de angulos
    Angulos = np.arange(0, 2*np.pi, DeltaAng)
    
    #Bucle de calculo de las coordenadas cartesianas
    for i in range(len(Angulos)):
        #Obtenemos el valor de ro
        ro = ListaPolar[i]
        alpha = Angulos[i] #Lo ponemos negativo para que sea en sentido anti-horario
            
        #Calculamos las coordenadas cartesianas teniendo en cuenta el limite de medida del sensor
        if RangoMin < ro < RangoMax:
            x = ro * np.cos(alpha)
            y = ro * np.sin(alpha)
            ListaCart.append([x, y])
    
    return ListaCart

#Creamos la clase Segmento 

class Segmento:
    """Clase que representa un segmento en un plano"""
    
    def __init__(self, puntoA, puntoB, idxA, idxB):
        """Constructor de la clase"""
        #Puntos extremos del segmento
        self.p0 = puntoA
        self.p1 = puntoB
        #Indices de los puntos
        self.idx0 = idxA
        self.idx1 = idxB
        
        #Tamaño del segmento
        self.tamano = self.length()
        
        #Vector director del segmento
        self.vector = [puntoB[0] - puntoA[0], puntoB[1] - puntoA[1]] / self.tamano
        
        #Vector normal al segmento
        self.normal = np.array([-self.vector[1], self.vector[0]])
        
        #Ecuacion de la recta
        self.ecuacion = self.CalcularEcuacion()
        
        
    def length(self):
        """Calcula el tamano del segmento"""
        return np.sqrt((self.p1[0] - self.p0[0])**2 + (self.p1[1] - self.p0[1])**2)
    
    def distancia(self, punto):
        """Calcula la distancia de un punto al segmento"""
        #Calculamos el vector director del segmento
        v = [self.p1[0] - self.p0[0], self.p1[1] - self.p0[1]]
        
        #Calculamos el vector director del punto al punto inicial del segmento
        w = [punto[0] - self.p0[0], punto[1] - self.p0[1]]
        
        #Calculamos el producto escalar
        escalar = v[0]*w[0] + v[1]*w[1]
        
        #Calculamos la proyeccion del punto sobre el segmento
        proy = escalar/(self.length()**2)
        
        #Calculamos el punto proyectado
        p = [self.p0[0] + proy*v[0], self.p0[1] + proy*v[1]]
        
        #Calculamos la distancia
        return np.sqrt((punto[0] - p[0])**2 + (punto[1] - p[1])**2)
    
    def angulo(self, segmento):
        """Calcula el angulo entre dos segmentos"""
        #Calculamos el vector director del primer segmento
        v1 = [self.p1[0] - self.p0[0], self.p1[1] - self.p0[1]]
        
        #Calculamos el vector director del segundo segmento
        v2 = [segmento.p1[0] - segmento.p0[0], segmento.p1[1] - segmento.p0[1]]
        
        #Calculamos el producto escalar
        escalar = v1[0]*v2[0] + v1[1]*v2[1]
        
        #Calculamos los modulos de los vectores
        mod1 = np.sqrt(v1[0]**2 + v1[1]**2)
        mod2 = np.sqrt(v2[0]**2 + v2[1]**2)
        
        #Calculamos el angulo
        return np.arccos(escalar/(mod1*mod2))
    
    def CalcularEcuacion(self):
        """Funcion que vamos a usar para calcular la ecuacion de la recta"""
        
        #Calculamos la pendiente
        m,n = np.polyfit([self.p0[0], self.p1[0]], [self.p0[1], self.p1[1]], 1)
        
        return [m, n]
        
    def __str__(self):
        """Funcion que usamos para imprimir el segmento"""
        return 'Segmento: p0: ' + str(self.p0) + ' p1: ' + str(self.p1)


class SplitAndMerge:
    def __init__(self, dist=0.09, angle=0.0, purge_pts = 8, purge_len = 0.3):
        self.d_th = dist        # Threshold distance (split)
        self.a_th = angle       # Threshold angle (merge)
        self.pur_pts = purge_pts  # Min number of points (purge)
        self.pur_len = purge_len  # Min segment length (purge)
        
        
    def split(self, Pts):
        """Funcion que usamos para dividir los segmentos (split)"""
        #print('Splitting...')
        #Definimos la lista de segmentos
        Segmentos = []
        
        #Inicializamos la pila de segmentos
        pila = queue.LifoQueue()
        
        #Anadimos todos los puntos a cola 
        pila.put([0, len(Pts)-1])
        
        #Bucle para mientras la pila no este vacia
        while not pila.empty():
            #Obtenemos los puntos
            idx0, idx1 = pila.get()
            
            #Creamos el segmento
            seg = Segmento(Pts[idx0], Pts[idx1], idx0, idx1)
            
            #Calculamos la distancia del resto de puntos al segmento
            dist_max = 0
            idx_split = -1
            
            for i in range(idx0, idx1):
                dist = seg.distancia(Pts[i])
                if dist > dist_max:
                    dist_max = dist
                    idx_split = i
                    
            if dist_max > self.d_th and idx_split != -1:
                #Anadimos los puntos a la pila
                pila.put([idx0, idx_split])
                pila.put([idx_split, idx1])
                
            else:
                Segmentos.append(seg)
        
        #print('Splitting done')
        return Segmentos

    def merge(self, segs_in):
        """Funcion que usamos para unir los segmentos (merge)"""
        
        #print('Merging...')
        #Definimos la lista de segmentos
        Segmentos = []
        
        #Bucle 
        while len(segs_in) > 0:
            #Obtenemos el primer segmento
            seg0 = segs_in.pop(0)
            if len(Segmentos) == 0:
                Segmentos.append(seg0)
            else:
                #Obtenemos el segmento previo 
                seg1 = Segmentos[-1]
                
                #Calculamos el angulo entre los segmentos
                angle = seg1.angulo(seg0)
                
                if angle < self.a_th:
                    #Unimos los segmentos
                    nuevo = Segmento(seg0.p0, seg1.p1, seg0.idx0, seg1.idx1)
                    Segmentos[-1] = nuevo
                    
                else:
                    #Anadimos el segmento a la lista
                    Segmentos.append(seg0)
        
        #print('Merging done')
        return Segmentos

    def purge(self, segs_in):
        """Funcion que usamos para purgar los segmentos (purge)"""
        
        #print("Purging...")
        #Definimos la lista de segmentos
        Segmentos = []
        
        #Bucle para eliminar los segmentos que tengas pocos puntos o longitud pequena
        for seg in segs_in:
            #print("Cantidad de puntos: ", seg.idx1 - seg.idx0, "Tamano del segmento: ", seg.length())
            if seg.idx1 - seg.idx0 >= self.pur_pts and seg.length() > self.pur_len:
                Segmentos.append(seg)
        
        #print('Purging done')
        return Segmentos
    
    def __call__(self, Puntos):
        """Funcion que ejecuta el algoritmo"""
        
        #Dividimos los segmentos
        segs = self.split(Puntos)
        #print('Numero de segmentos tras split: ', len(segs))
        
        #Unimos los segmentos
        segs = self.merge(segs)
        
        #Purgamos los segmentos
        segs = self.purge(segs)
        
        return segs
    
    def plotSegments(self, segmentList, debug=False):
        """Funcion que usamos para dibujar los segmentos"""
        
        global RangoMax
        
        #plt.clf()
        
        for s in segmentList:
            x = (s.p0[0], s.p1[0])
            y = (s.p0[1], s.p1[1])
            plt.plot(x, y, marker = 'o', color='r')
        plt.draw()
        plt.pause(0.001)
        if debug:
            foo = input('Press a key and hit enter to continue')
        
        plt.xlim(-RangoMax, RangoMax)
        
    def plot_points(self, Pts):
        """Funcion que usamos para dibujar los puntos"""
        
        global RangoMax
        
        #Dibujamos los puntos
        
        pt = Pts[0]
        plt.plot(pt[0],pt[1], marker='.', color='g')
        
        for ii in range(1, len(Pts)-1):
            pt = Pts[ii]
            plt.plot(pt[0],pt[1], marker='.', color='k')
            
        pt = Pts[-1]
        plt.plot(pt[0],pt[1], marker='.', color='g')
        
        plt.draw()
        plt.pause(0.01)
        
        
        
        #Definimos unos limites
        plt.xlim(-RangoMax, RangoMax)

    def plot(self, Pts, Segs):
        """Funcion que usamos para dibujar los segmentos y los puntos conjuntamente"""
        
        #print('Plotting...')
        
        global RangoMax
        
        #Eliminamos la figura actual
        plt.clf()
        
        #Definimos unos limites
        plt.xlim(-RangoMax, RangoMax)
        plt.ylim(-RangoMax, RangoMax)
        
        #Dibujamos los puntos
        self.plot_points(Pts)
        self.plotSegments(Segs)



#TODO: Definimos la variables globales

#Valores intrinsecos del sensor
DeltaAng = 0.0175
RangoMin = 0.12
RangoMax = 3.5
