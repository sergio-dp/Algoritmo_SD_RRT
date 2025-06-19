# EN ESTE ARCHIVO, SE DEFINEN LAS FUNCIONES RELACIONAS CON LA VISUALIZACIÓN DEL MAPA
# Y DETECCIÓN DE LOS OBSTÁCULOS Y COLISIONES
import numpy as np
import random
import math
import auxiliar

# MAPA PLANTA 1
mapa_1 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (0,95,75,10),     # Habitacion 1
         (135,95,65,10),  # Habitacion 1
         (200,95,10,155),  # Habitacion 1
         (0,245,200,10),   # Habitacion 1/2
         (200,250,10,155), # Habitacion 2
         (0,395,75,10),   # Habitacion 2
         (135,395,65,10),  # Habitacion 2
         (0,495,185,10),   # Habitacion 3
         (245,495,10,105), # Habitacion 3/4
         (255,495,180,10), # Habitacion 4
         (495,495,10,105), # Habitacion 4/5
         (565,495,180,10), # Habitacion 5
         (745,495,10,105), # Habitacion 5/6
         (815,495,185,10), # Habitacion 6
         (800,395,200,10), # Habitacion 7
         (800,245,10,90), # Habitacion 7
         (800,245,200,10), # Habitacion 7/8
         (800,165,10,90), # Habitacion 8
         (800,95,200,10),  # Habitacion 8
         (300,125,170,30), # Centro arriba izquierda
         (300,125,30,100), # Centro arriba izquierda
         (530,125,170,30), # Centro arriba derecha
         (670,125,30,100), # Centro arriba derecha
         (300,354,170,30), # Centro abajo izquierda
         (300,284,30,100), # Centro abajo izquierda
         (530,354,170,30), # Centro abajo derecha
         (670,284,30,100), # Centro abajo derecha
)
# MAPA PLANTA 2
mapa_2 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (0,240,150,10),
         (0,350,45,10),
         (105,350,45,10),
         (140,240,10,120),
         (245,355,10,245),
         (245,0,10,160),
         (245,220,10,80),
         (245,150,100,10),
         (245,290,100,10),
         (340,90,10,210),
         (340,90,60,10),
         (390,0,10,40),
         (390,90,10,200),
         (460,280,10,100),
         (460,280,270,10),
         (720,0,10,30),
         (720,90,10,290),
         (460,370,55,10),
         (565,370,425,10),
         (720,180,155,10),
         (935,180,65,10),

         (545,430,120,10),
         (545,430,10,170),
         (655,490,10,110),
)
# MAPA LABERINTO
mapa_3 = (
         (0,0,1000,10),
         (0,0,10,600),
         (990,0,10,600),
         (0,590,1000,10),
         (148,0,4,62),
         (148,58,154,4),
         (298,58,4,64),
         (48,0,4,60),
         (48,58,54,4),
         (98,58,4,124),
         (0,118,52,4),
         (0,178,52,4),
         (48,118,4,64),
         (98,118,154,4),
         (198,118,4,64),
         (198,178,54,4),
         (248,178,4,64),
         (48,238,204,4),
         (148,178,4,64),
         (48,238,4,124),
         (48,358,254,4),
         (98,298,4,64),
         (198,238,4,64),
         (148,298,104,4),
         (298,358,4,124),
         (248,358,4,64),
         (148,478,154,4),
         (0,418,204,4),
         (48,478,4,124),
         (98,478,4,64),
         (98,538,254,4),
         (348,0,4,242),
         (298,178,54,4),
         (298,238,154,4),
         (298,238,4,64),
         (448,238,4,244),
         (448,478,54,4),
         (348,298,4,244),
         (348,298,54,4),
         (398,298,4,244),
         (398,538,204,4),
         (448,0,4,62),
         (398,58,54,4),
         (398,58,4,64),
         (398,118,254,4),
         (498,118,4,304),
         (398,178,104,4),
         (548,0,4,62),
         (498,58,54,4),
         (548,178,4,364),
         (548,178,154,4),
         (548,298,104,4),
         (548,418,204,4),
         (598,478,104,4),
         (648,478,4,124),
         (598,58,154,4),
         (698,58,4,64),
         (748,58,4,94),
         (598,358,204,4),
         (748,358,4,184),
         (798,358,4,244),
         (698,538,54,4),
         (598,238,154,4),
         (698,238,4,64),
         (748,208,4,94),
         (748,298,154,4),
         (848,298,4,244),
         (898,298,4,304),
         (898,478,54,4),
         (798,0,4,242),
         (798,58,154,4),
         (798,238,54,4),
         (848,118,154,4),
         (848,118,4,64),
         (898,178,54,4),
         (898,178,4,64),
         (898,238,54,4),
         (948,238,4,184),
         (948,418,54,4),
         (948,538,54,4),
)
# MAPA EN FORMA DE T
mapa_4 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (150,80,750,20),
         (500,70,20,430),
         (50,500,500,20)
)
# MAPA FÁCIL
mapa_5 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (460,250,80,100),
)

class MapData:
    def __init__(self, map, description, init, goal):
        self.map = map
        self.init = init
        self.goal = goal
        self.description = description

maps = (MapData(mapa_5, "Mapa fácil", (360, 300), (660, 300)),
        MapData(mapa_4, "Mapa T", (60, 90), (850, 500)),
        MapData(mapa_1, "Plano 1", (50, 200), (950, 550)),
        MapData(mapa_2, "Plano 2", (30, 570), (790, 340)),
        MapData(mapa_3, "Laberinto", (25, 30), (975, 570)))

# DISTANCIA ENTRE 2 PUNTOS
def distancia_entre_puntos(p1,p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5

# MODULO DE UN VECTOR
def modulo_vector(v):
    return (v[0]**2+v[1]**2)**0.5

# DISTANCIA ENTRE UN PUNTO Y UN SEGMENTO
def distancia_punto_segmento(p,s1,s2):
    ab = (s2[0]-s1[0], s2[1]-s1[1])
    ap = (p[0]-s1[0],p[1]-s1[1])
    bp = (p[0]-s2[0],p[1]-s2[1])
    proyeccion = ab[0]*ap[0]+ab[1]*ap[1]
    m = ab[0]**2+ab[1]**2
    if proyeccion <= 0:
        return modulo_vector(ap), s1
    if proyeccion >= m:
        return modulo_vector(bp), s2
    pi = (int(s1[0]+proyeccion*ab[0]/m), int(s1[1]+proyeccion*ab[1]/m))
    return modulo_vector((p[0]-pi[0],p[1]-pi[1])), pi

# FUNCIÓN QUE CALCULA LA DISTANCIA ENTRE UN PUNTO Y UN OBSTÁCULO
def distancia_punto_obstaculo(punto, rect):
    px, py = punto
    rx, ry, ancho, alto = rect
    rx2 = rx + ancho
    ry2 = ry + alto

    # Calcular la distancia mínima en x
    dx = 0
    if px < rx:
        dx = rx - px
    elif px > rx2:
        dx = px - rx2

    # Calcular la distancia mínima en y
    dy = 0
    if py < ry:
        dy = ry - py
    elif py > ry2:
        dy = py - ry2

    # Devuelvo la distancia euclídea
    return ((dx)**2+(dy)**2)**0.5

# CLASE QUE ALMACENA TODOS LOS DATOS DEL MAPA
class Mapa_base:
    # CONSTRUCTOR
    def __init__(self, w,h):
        self.altura, self.anchura = h,w # Guarda el alto y ancho del mapa

        # CONFIGURACIÓN DE LA VENTANA
        auxiliar.pygame.display.set_caption('Algoritmos de planificación basados en muestreo')
        self.canvas = auxiliar.pygame.display.set_mode((self.anchura, self.altura))

        # LISTA DE OBSTACULOS
        self.lista_obstaculos = [] # Inicialmente, lista vacía
    
    # FUNCION QUE GENERA UN OBSTACULO ALEATORIO
    def crear_obstaculo_random(self):
        w = int(random.uniform(auxiliar.minimo_tamaño_obstaculo,auxiliar.varianza_obstaculos)) # Ancho
        h = int(random.uniform(auxiliar.minimo_tamaño_obstaculo,auxiliar.varianza_obstaculos)) # Alto
        upx = int(random.uniform(0,self.anchura-w)) # Coordenada x dentro del mapa
        upy = int(random.uniform(0,self.altura-h)) # Coordenada y dentro del mapa
        return auxiliar.pygame.Rect(upx, upy, w, h)
 
    # FUNCION QUE GENERA UN MAPA ALEATORIO
    def crear_mapa_aleatorio(self,n, puntos = []):
        self.lista_obstaculos = [auxiliar.pygame.Rect(0,0,1000,10),auxiliar.pygame.Rect(0,0,10,600),auxiliar.pygame.Rect(990,0,10,600),auxiliar.pygame.Rect(0,590,1000,10)]
        for i in range(0, n):
            self.lista_obstaculos.append(self.crear_obstaculo_random())
        for p in puntos:
            self.eliminar_obstaculo(p)

    # FUNCION QUE CARGA UN MAPA ESPECIFICO
    def cargar_mapa(self, rectangulos, puntos = []):
        self.lista_obstaculos = []
        for r in rectangulos:
            self.lista_obstaculos.append(auxiliar.pygame.Rect(*r))
        for p in puntos:
            self.eliminar_obstaculo(p)

    # FUNCION QUE ELIMINA TODOS LOS OBSTACULOS EN UN PUNTO
    def eliminar_obstaculo(self, punto):
        self.lista_obstaculos = [obs for obs in self.lista_obstaculos if not obs.collidepoint(punto)] # collidepoint devuelve True si point esta dentro de obs

    # FUNCION QUE DIBUJA LOS OBSTACULOS DEL MAPA
    def dibujar(self):
        self.canvas.fill(auxiliar.blanco) # Pone el fondo en blanco
        for ob in self.lista_obstaculos:
            auxiliar.pygame.draw.rect(self.canvas, auxiliar.gris, ob) # Dibuja cada obstaculo en gris

    # FUNCION QUE DIBUJA LOS PUNTOS INICIAL Y FINAL DE LA TRAYECTORIA
    def dibujar_origen_destino(self, origen, destino):
        auxiliar.pygame.draw.circle(self.canvas, auxiliar.rojo, origen, 3*auxiliar.radio_nodo, 3*auxiliar.radio_nodo) # Punto inicial en rojo
        auxiliar.pygame.draw.circle(self.canvas, auxiliar.verde, destino, 3*auxiliar.radio_nodo, 3*auxiliar.radio_nodo) # Punto final en verde

    # FUNCION QUE VERIFICA SI UNA LINEA ENTRE 2 PUNTOS (p1 y p2) CHOCA CON ALGUN OBSTACULO
    def comprobar_segmento(self, p1, p2):
        for ob in self.lista_obstaculos:
            if ob.clipline(p1,p2):
                return False
        return True
    
    # FUNCIÓN QUE VERIFICA SI UN PUNTO CHOCA CON UN OBSTÁCULO
    def comprobar_punto(self, punto):
        if punto[0] < 0 or punto[0] > self.anchura or punto[1] < 0 or punto[1] > self.altura:
            return False
        for ob in self.lista_obstaculos:
            if ob.collidepoint(punto):
                return False
        return True
    
    # FUNCIÓN QUE CALCULA LA DISTANCIA ENTRE UN PUNTO Y UN OBSTÁCULO
    def distancia_minima_a_obstaculos(self, punto):
        return min(distancia_punto_obstaculo(punto, rect) for rect in self.lista_obstaculos)
    
    # FUNCION QUE ENCUENTRA EL PUNTO MAS LEJANO DE UN SEGMENTO SIN COLISION
    def punto_de_parada(self, p1, p2):
        n = int(1+distancia_entre_puntos(p1,p2)//auxiliar.paso_planificador) # Calcula el numero de puntos intermedios entre p1 y p2
        puntos = [(int((p2[0]*i+p1[0]*(n-i))/n),int((p2[1]*i+p1[1]*(n-i))/n)) for i in range(0,n+1)] # Lista con dichos puntos intermedios
        pp =  p1  
        for p in puntos: 
            if not self.comprobar_punto(p): # Se da cuando un punto esta dentro de un obstaculo
                return pp, True
            pp = p
        return p2, False # Si ningun punto intermedio choca con obstaculo, devuelve el punto p2

    # FUNCION QUE GENERA UNA MUESTRA ALEATORIA EN EL MAPA
    def muestra_aleatoria(self):
        return int(random.uniform(0,self.anchura)), int(random.uniform(0,self.altura))
    
    # FUNCION QUE GENERA UNA MUESTRA RESTRNGIDA A UNA HIPERELIPSOIDE
    def muestra_restringida(self, p1, p2, c_max, c_min):
        if c_max == math.inf:
            alpha = self.muestra_aleatoria() # Generamos un punto aleatorio
            r1 = math.inf
            r2 = math.inf
        else:
            # Ejes de la elipse y punto central
            r1 = 0.5*c_max # Longitud del semi-eje mayor
            r2 = 0.5*(c_max**2 - c_min**2)**0.5 # Longitud del semi-eje menor
            centro_elipse = ((p1[0] + p2[0])*0.5,(p1[1] + p2[1])*0.5)

            # Matriz de rotacion (para orientar segun la direccion de la elipse)
            angulo = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            R = ((math.cos(angulo), -math.sin(angulo)), (math.sin(angulo), math.cos(angulo)))

            # La muestra generada debe estar dentro del mapa
            while True:
                # Generamos un numero aleatorio dentro de la circunferencia unidad
                t = random.uniform(0, 2 * math.pi)
                r = random.uniform(0, 1)**0.5
                muestra_unidad = (r * math.cos(t), r * math.sin(t))

                # Aplicamos la ecuacion matematica
                alpha = (centro_elipse[0]+r1*muestra_unidad[0]*R[0][0]+r2*muestra_unidad[1]*R[0][1],
                         centro_elipse[1]+r1*muestra_unidad[0]*R[1][0]+r2*muestra_unidad[1]*R[1][1])
                if 0 < alpha[0] < self.anchura and 0 < alpha[1] < self.altura:
                    break
        return alpha, r1, r2