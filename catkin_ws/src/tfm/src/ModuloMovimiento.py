class PDController:
    def __init__(self, kpDistancia, kdDistancia, kpAngulo, kdAngulo , Ponderacion = 0.5):
        """Constructor de la clase"""
        #Constantes del controlador
        self.kpDistancia = kpDistancia
        self.kdDistancia = kdDistancia
        self.kpAngulo = kpAngulo
        self.kdAngulo = kdAngulo
        
        #Variables del controlador
        self.previous_errorDistancia = 0
        self.previous_errorAngulo = 0
        
        #Valor de importancia a distancia o angulo
        self.Ponderacion = Ponderacion 

    def compute(self, errorDistancia , errorAngulo , dt):
        """Calcula la salida del controlador"""
        
        #TODO: Controlador PD para distancia
        
        #Deribada del error
        derivativeDistancia = (errorDistancia - self.previous_errorDistancia) / dt
        
        #Calculo de la salida 
        outputDistancia = self.kpDistancia * errorDistancia + self.kdDistancia * derivativeDistancia
        
        
        #TODO: Controlador PD para angulo
        
        #Deribada del error
        derivativeAngulo = (errorAngulo - self.previous_errorAngulo) / dt
        
        #Calculo de la salida
        outputAngulo = self.kpAngulo * errorAngulo + self.kdAngulo * derivativeAngulo
        
        #TODO: Calculo de la salida final
        
        output = self.Ponderacion * outputDistancia + (1 - self.Ponderacion) * outputAngulo
        
        
        #TODO: Actualizacion de variables
        self.previous_errorDistancia = errorDistancia
        self.previous_errorAngulo = errorAngulo
        
        return output


def CalcularVelocidades(Estado, DistanciaX = None, DistanciaY = None , Angulo = None, DistanciaObjetivo = 0.5):
    """Funcion para calcular las velocidades en funcion de la distancia"""
    
    global pd, DistanciaObjetivoY, AnguloObjetivo 
    
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
        Vang = pd.compute(errorY, errorAngulo, 0.1)

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


#Definimos los valores objetivo
DistanciaObjetivoY = 0.5
AnguloObjetivo = 0.0
DistaciaObjetivoX= 1

#Definimos un controlador PD
pd = PDController(0.35, 0.4, 0.1, 0.1, 0.6)