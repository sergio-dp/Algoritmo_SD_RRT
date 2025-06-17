import auxiliar
import mapas
import math
from collections import deque

# FUNCION ESPECÍFICA PARA EL RRT* QUE CALCULA EL RADIO DE BUSQUEDA DE NODOS VECINOS
def radio_optimo(n, tamaño_mapa = auxiliar.tamaño_mapa[0], gamma = 0.7):
    return gamma*tamaño_mapa*(math.log(n+1)/(n+1))**0.5

class algoritmo:
    def __init__(self, mapa, origen, destino, seleccion_mapa, dist_seguridad, muestreo_global, muestreo_restringido):
        self.arbol = Arbol(origen, destino, mapa.canvas, mapa)
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.iteracion = 1 # Vector donde cada nodo almacena a su padre

    def iteraciones(self, max_iter):
        arbol = self.arbol
        mapa = self.mapa
        destino = self.destino
        for i in range(max_iter):
            alpha = mapa.muestra_aleatoria() # Generamos un punto aleatorio
            if not self.iteracion % 100 and not destino in arbol.padres: # Si no se ha encontrado camino ya, cada 100 iteraciones intentamos unir el árbol al punto destino
                alpha = destino
            qn, arista = arbol.punto_mas_cercano(alpha) # Punto y arista mas cercanos del arbol a alpha
            qs, _ = mapa.punto_de_parada(qn, alpha) # Punto de parada al trazar la arista desde qn hasta alpha
            if qs != qn: # Si la arista tiene longitud
                arbol.cablear_arbol(qs, qn, arista, mapa)
            self.iteracion += 1 # Nueva iteracion realizada
            if destino in arbol.padres: # El objetivo pertenece al arbol
                arbol.dibujar_camino() # Dibujamos el mejor camino encontrado 
        return False, arbol.long_camino, arbol.dist_minima, len(arbol.padres)+1

class Arbol:
    # CONSTRUCTOR
    def __init__(self, origen, destino, canvas, mapa):
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.long_camino = math.inf # Para conocer la longitud de un camino una vez obtenido
        self.padres = {} # Vector donde cada nodo almacena a su padre
        self.canvas = canvas # Canvas donde se dibuja el árbol
        self.coste_nodo = {origen:0} # Vector donde cada nodo almacena el coste acumulado de llegar desde el origen
        self.coste_arista = {} # Vector donde cada nodo almacena el coste de llegar desde el padre (distancia arista)
        self.dist_minima = math.inf # Distancia mínima a un obstáculo en los nodos del camino

    # FUNCION DE DIBUJO GLOBAL
    def dibujar(self):
        for p2 in self.padres: # Dibuja las aristas del arbol
            self.dibujar_arista(p2)
            self.dibujar_nodos(p2)
        if self.destino in self.padres: # Si existe camino entre el origen y el objetivo, lo señalamos de rojo
            self.dibujar_camino()

    # FUNCION PARA DIBUJAR LAS ARISTAS DEL ÁRBOL
    def dibujar_arista(self, p2):
        auxiliar.pygame.draw.line(self.canvas, auxiliar.color_arista, self.padres[p2][:2] ,p2[:2], auxiliar.grosor_arista)

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
            #self.dist_minima = min(self.dist_minima, self.dist_obstaculo[nodo_actual])
            self.dist_minima = min(self.dist_minima, self.mapa.distancia_minima_a_obstaculos(nodo_actual))

        # También evaluamos el último nodo del camino
        ultimo_nodo = camino[-1]
        self.dist_minima = min(self.dist_minima, self.mapa.distancia_minima_a_obstaculos(ultimo_nodo))

    # FUNCIÓN PARA OBTENER EL CAMINO ENTRE ORIGEN Y DESTINO
    def obtener_camino(self):
        camino = deque()
        nodo = self.destino
        while nodo in self.padres:
            camino.appendleft(nodo)
            nodo = self.padres[nodo]
        camino.appendleft(self.origen)
        return list(camino)

    # FUNCION QUE DEVUELVE EL PUNTO MAS CERCANO DEL ARBOL A OTRO DADO
    def punto_mas_cercano(self, p):
        qn = self.origen # Inicialmente, el punto mas cercano es el nodo padre del árbol
        minima_distancia = mapas.distancia_entre_puntos(qn,p)
        arista_punto_mas_cercano = None
        for hijo, padre in self.padres.items(): # p1 y p2 son padre e hijo que forman una arista
            d,q = mapas.distancia_punto_segmento(p,padre,hijo) # Distancia entre p y la arista p1-p2
            if d < minima_distancia: # Si la distancia es la menor
                minima_distancia, qn = d, q
                if qn == padre or qn == hijo: # Si el punto mas cercano es directamente un extremo de la arista
                    arista_punto_mas_cercano = None
                else: # Si el punto mas cercano esta dentro de la arista
                    arista_punto_mas_cercano = hijo # Guardamos el hijo, de forma que el padre es directamente self.padres[hijo]
        return qn, arista_punto_mas_cercano

    # ESTA FUNCIÓN SIRVE PARA RECABLEAR EL ÁRBOL ALREDEDOR DE qs (punto parada)
    def cablear_arbol(self, qs, qn, arista, mapa):
        repintar_mapa = False
        vecinos = self.obtener_vecinos(qs, radio_optimo(len(self.padres)))
        distancia_minima = mapas.distancia_entre_puntos(qs,qn)
        if arista:
            coste_minimo = distancia_minima + self.coste_nodo[self.padres[arista]] + mapas.distancia_entre_puntos(self.padres[arista],qn)
        else:
            coste_minimo = distancia_minima + self.coste_nodo[qn]

        # Primer paso, vemos el punto mas optimo al que conectar el punto de parada
        for vecino in vecinos:
            if vecino[3] < coste_minimo and mapa.comprobar_segmento(vecino[0],qs):
                self.añadir_arista(vecino[0], qs, None)
                vecinos.remove(vecino)
                break
        else: # El coste mas optimo es conectar qs al arbol a traves de qn
            self.añadir_arista(qn, qs, arista)

        # Segundo paso, intentamos reconectar a traves de qs para reducir el coste
        coste_minimo = self.coste_nodo[qs]
        for vecino in vecinos:
            # Si el coste de llegar al vecino a traves de qs (peso qs-origen + distancia vecino-qs) es menor que el coste actual
            if coste_minimo + vecino[1] < vecino[2] and mapa.comprobar_segmento(vecino[0],qs):
                self.cambiar_padre(vecino[0],qs) # Cambiamos el padre
                repintar_mapa = True
        if repintar_mapa:
                mapa.dibujar()
                self.dibujar()
                mapa.dibujar_origen_destino(self.origen, self.destino)

    # ESTA FUNCION SIRVE PARA ENCONTRAR NODOS CERCANOS A UN PUNTO p
    def obtener_vecinos(self,qs,r):
        vecinos = [] # En este vector se almacenaran los nodos cercanos
        for n in self.padres:
            d = mapas.distancia_entre_puntos(qs,n) # Distancia de p al nodo n
            if d < r: # Si el nodo esta centro de un circulo de radio r
                # Cada nodo se guarda en el formato (nodo, distancia, coste origen-nodo n, coste origen-punto p)
                vecinos.append((n, d, self.coste_nodo[n], d + self.coste_nodo[n]))
        vecinos.sort(key=lambda x:x[3])
        return vecinos

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

        # Actualizamos todos los costes
        if arista:
            self.calcular_coste(padre)
        self.calcular_coste(hijo)
        if arista:
            self.calcular_coste(arista)

    # ESTA FUNCION SIRVE PARA CALCULAR LOS COSTES ASOCIADOS A UN NODO NUEVO
    def calcular_coste(self, p):
        if not p in self.padres: # Si el nodo no esta en el arbol, no hace nada
            return
        self.coste_arista[p] = mapas.distancia_entre_puntos(self.padres[p],p) # Distancia entre un nodo y su padre
        self.coste_nodo[p] = self.coste_arista[p] + self.coste_nodo[self.padres[p]] # Coste desde el origen al nodo

    # ESTA FUNCION SIRVE PARA CAMBIAR EL PADRE DE UN NODO
    def cambiar_padre(self, p, nuevo_padre):
        if not p in self.padres: # Si el nodo no esta en el arbol, no hace nada
            return
        self.padres[p] = nuevo_padre # Cambia el padre
        self.actualizar_coste(p, True) # Actualiza el coste

    # ESTA FUNCION SIRVE PARA ACTUALIZAR LOS COSTES DEL ARBOL AL RECABLEAR EL ARBOL
    def actualizar_coste(self, p, padre_cambiado = False):
        if not p in self.padres: # Si el nodo no esta en el arbol, no hace nada
            return
        if padre_cambiado:
            self.coste_arista[p] = mapas.distancia_entre_puntos(self.padres[p],p)
        self.coste_nodo[p] = self.coste_arista[p] + self.coste_nodo[self.padres[p]]
        # Cogemos los nodos que tienen a p como padre
        hijos = [nodo for nodo, padre in self.padres.items() if padre == p]
        for hijo in hijos:
            self.actualizar_coste(hijo)   