import auxiliar
import mapas
import math
from collections import deque

class algoritmo:
    def __init__(self, mapa, origen, destino, seleccion_mapa, dist_seguridad, muestreo_global, muestreo_restringido):
        self.arbol_a = Arbol(origen, auxiliar.azul, mapa.canvas, mapa) # El arbol A se expande desde el origen
        self.arbol_b = Arbol(destino, auxiliar.verde, mapa.canvas, mapa) # El arbol B se expande desde el objetivo
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.iteracion = 1 # Vector donde cada nodo almacena a su padre

    def iteraciones(self, max_iter):
        for i in range(max_iter):
            alpha = self.mapa.muestra_aleatoria() # Generamos un punto aleatorio
            qn_a, arista_a = self.arbol_a.punto_mas_cercano(alpha) # Punto y arista mas cercanos del arbol A a alpha
            qs_a, _ = self.mapa.punto_de_parada(qn_a, alpha) # Punto de parada al trazar la arista desde qn_a hasta alpha
            if qs_a != qn_a: # Si la arista tiene longitud
                self.arbol_a.añadir_arista(qn_a, qs_a, arista_a) # Introducimos la arista al arbol A

                # Una vez introducida la arista, intentamos unir ambos arboles    
                qn_b, arista_b = self.arbol_b.punto_mas_cercano(qs_a) # Punto y arista mas cercanos del arbol B al nuevo punto del arbol A
                qs_b, _ = self.mapa.punto_de_parada(qn_b, qs_a) # Punto de parada al trazar la arista que intenta unir ambos arboles
                if qs_b != qn_b: # Si la arista tiene longitud
                    self.arbol_b.añadir_arista(qn_b, qs_b, arista_b) # Introducimos la arista al arbol B
                if qs_b == qs_a: # Si logramos unir ambos arboles
                    self.arbol_a.dibujar_camino(qs_a) # Pintamos la parte del camino perteneciente al arbol A
                    self.arbol_b.dibujar_camino(qs_b) # Pintamos la parte del camino perteneciente al arbol B
                    print("ÉXITO en la iteración: ", self.iteracion)
                    print("Longitud de la ruta: ", self.arbol_a.long_camino+self.arbol_b.long_camino)
                    print("Distancia mínima a un obstáculo: ", min(self.arbol_a.dist_minima, self.arbol_b.dist_minima))
                    print("Número de nodos del árbol: ", len(self.arbol_a.padres)+len(self.arbol_b.padres)+2)
                    return True, self.arbol_a.long_camino+self.arbol_b.long_camino, min(self.arbol_a.dist_minima, self.arbol_b.dist_minima), len(self.arbol_a.padres)+len(self.arbol_b.padres)+2

            # El algoritmo esta pensado para que ambos arboles crezcan a la par. Si A se hace mas grande que B, los intercambiamos
            if len(self.arbol_a.padres) > len(self.arbol_b.padres) : 
                self.arbol_a, self.arbol_b = self.arbol_b, self.arbol_a

            self.iteracion += 1 # Nueva iteracion realizada
            print("Iteración: ", self.iteracion)
        return False, math.inf, math.inf, len(self.arbol_a.padres)+len(self.arbol_b.padres)+2

class Arbol:
    # CONSTRUCTOR
    def __init__(self, origen, color, canvas, mapa):
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.long_camino = math.inf # Para conocer la longitud de un camino una vez obtenido
        self.color = color # Color para la representacion grafica
        self.padres = {} # Vector donde cada nodo almacena a su padre
        self.canvas = canvas # Canvas donde se dibuja el árbol

    # FUNCION PARA DIBUJAR LAS ARISTAS DEL ÁRBOL
    def dibujar_arista(self, p2):
        auxiliar.pygame.draw.line(self.canvas, self.color, self.padres[p2][:2] ,p2[:2], auxiliar.grosor_arista)

    # FUNCION PARA DIBUJAR LOS NODOS DEL ÁRBOL
    def dibujar_nodos(self, p2):
        auxiliar.pygame.draw.circle(self.canvas, self.color, p2[:2], auxiliar.radio_nodo, auxiliar.radio_nodo)

    # FUNCION PARA DIBUJAR EL CAMINO UNA VEZ ENCONTRADO
    def dibujar_camino(self, destino):
        camino = self.obtener_camino(destino)
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
    def obtener_camino(self, destino):
        camino = deque()
        nodo = destino
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
            d,q = mapas.distancia_punto_segmento(p,p1,p2) # Distancia entre p y la arista p1-p2
            if d < minima_distancia: # Si la distancia es la menor
                minima_distancia, qn = d, q
                if qn == p1 or qn == p2: # Si el punto mas cercano es directamente un extremo de la arista
                    arista_punto_mas_cercano = None
                else: # Si el punto mas cercano esta dentro de la arista
                    arista_punto_mas_cercano = p2 # Guardamos el hijo, de forma que el padre es directamente self.tree[p2]
        return qn, arista_punto_mas_cercano