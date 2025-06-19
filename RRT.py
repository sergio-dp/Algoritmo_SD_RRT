import auxiliar
import mapas
import math
from collections import deque

class algoritmo:
    def __init__(self, mapa, origen, destino, seleccion_mapa, dist_seguridad, muestreo_global, muestreo_restringido):
        self.arbol = Arbol(origen, destino, mapa.canvas, mapa) # Creamos el arbol
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.iteracion = 1

    def iteraciones(self, max_iter):
        for i in range(max_iter):
            alpha = self.mapa.muestra_aleatoria() # Generamos un punto aleatorio
            if not self.iteracion % 100: # Cada 100 iteraciones intentamos unir el árbol al punto destino
                alpha = self.destino
            qn, arista = self.arbol.punto_mas_cercano(alpha) # Punto y arista mas cercanos del arbol a alpha
            qs, _ = self.mapa.punto_de_parada(qn, alpha) # Punto de parada al trazar la arista desde qn hasta alpha
            if qs != qn or qs == self.destino: # Si la arista tiene longitud
                self.arbol.añadir_arista(qn, qs, arista)
            if qs == self.destino: # En el caso de que la arista llegue hasta el punto objetivo, hemos terminado
                self.arbol.dibujar_camino()
                print("ÉXITO en la iteración: ", self.iteracion)
                print("Longitud de la ruta: ", self.arbol.long_camino)
                print("Distancia mínima a un obstáculo: ", self.arbol.dist_minima)
                print("Número de nodos del árbol: ", len(self.arbol.padres)+1)
                return True, self.arbol.long_camino, self.arbol.dist_minima, len(self.arbol.padres)+1
            self.iteracion += 1 # Nueva iteracion realizada
            print("Iteración: ", self.iteracion)
        return False, math.inf, math.inf, len(self.arbol.padres)+1

class Arbol:
    # CONSTRUCTOR
    def __init__(self, origen, destino, canvas, mapa):
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.long_camino = math.inf # Para conocer la longitud de un camino una vez obtenido
        self.padres = {origen: None} # Vector donde cada nodo almacena a su padre
        self.canvas = canvas # Canvas donde se dibuja el árbol
        self.dist_minima = math.inf # Distancia mínima a un obstáculo en los nodos del camino

    # FUNCION PARA DIBUJAR LAS ARISTAS DEL ÁRBOL
    def dibujar_arista(self, p2):
        auxiliar.pygame.draw.line(self.canvas, auxiliar.color_arista, self.padres[p2][:2], p2[:2], auxiliar.grosor_arista)

    # FUNCION PARA DIBUJAR LOS NODOS DEL ÁRBOL
    def dibujar_nodos(self, p2):
        auxiliar.pygame.draw.circle(self.canvas, auxiliar.color_nodo, p2[:2], auxiliar.radio_nodo, auxiliar.radio_nodo)

    # FUNCION PARA DIBUJAR EL CAMINO UNA VEZ ENCONTRADO
    def dibujar_camino(self):
        camino = self.obtener_camino()
        self.dist_minima = math.inf
        self.long_camino = 0

        # Recorremos el camino como pares consecutivos: (nodo1, nodo2)
        for i in range(len(camino) - 1):
            nodo_actual, nodo_siguiente = camino[i], camino[i + 1]
            auxiliar.pygame.draw.line(self.canvas, auxiliar.rojo, nodo_actual[:2], nodo_siguiente[:2], auxiliar.grosor_camino)
            self.long_camino += mapas.distancia_entre_puntos(nodo_actual, nodo_siguiente)
            self.dist_minima = min(self.dist_minima, self.mapa.distancia_minima_a_obstaculos(nodo_actual))

        # También evaluamos el último nodo del camino
        ultimo_nodo = camino[-1]
        self.dist_minima = min(self.dist_minima, self.mapa.distancia_minima_a_obstaculos(ultimo_nodo))
        auxiliar.pygame.display.update()

    # FUNCIÓN PARA OBTENER EL CAMINO ENTRE ORIGEN Y DESTINO
    def obtener_camino(self):
        camino = deque()
        nodo = self.destino
        while nodo in self.padres:
            camino.appendleft(nodo)
            nodo = self.padres[nodo]
        camino.appendleft(self.origen)
        return list(camino)

    # FUNCION QUE SIRVE PARA AÑADIR UNA ARISTA AL ARBOL
    def añadir_arista(self, padre, hijo, arista): # p1 es el padre, p2 es el hijo
        if hijo in self.padres: # Si p2 ya pertenece al arbol, no hacemos nada
            return False
        if arista and arista in self.padres and not padre in self.padres: # Este caso se da si vamos a conectar p2 a un punto medio de una arista
            self.padres[padre], self.padres[hijo], self.padres[arista] = self.padres[arista], padre , padre
            self.dibujar_nodos(padre)
        else: # Este caso se da si vamos a conectar p2 a un nodo del arbol
            self.padres[hijo] = padre # Si p2 no pertenece al arbol, p1 es su padre
        self.dibujar_arista(hijo)
        self.dibujar_nodos(hijo)

    # FUNCION QUE DEVUELVE EL PUNTO MAS CERCANO DEL ARBOL A OTRO DADO
    def punto_mas_cercano(self, p):
        qn =  self.origen # Inicialmente, el punto mas cercano es el nodo padre del árbol
        minima_distancia = mapas.distancia_entre_puntos(qn,p)
        arista_punto_mas_cercano = None
        for p2, p1 in self.padres.items(): # p1 y p2 son padre e hijo que forman una arista
            if p1 is None:
                continue
            
            d,q = mapas.distancia_punto_segmento(p,p1,p2) # Distancia entre p y la arista p1-p2
            if d < minima_distancia: # Si la distancia es la menor
                minima_distancia, qn = d, q
                if qn == p1 or qn == p2: # Si el punto mas cercano es directamente un extremo de la arista
                    arista_punto_mas_cercano = None
                else: # Si el punto mas cercano esta dentro de la arista
                    arista_punto_mas_cercano = p2 # Guardamos el hijo, de forma que el padre es directamente self.tree[p2]
        return qn, arista_punto_mas_cercano