import auxiliar
import mapas
import math

# FUNCIÓN QUE DETERMINA EL RADIO DE BÚSQUEDA DE VECINOS CERCANOS
def radio_busqueda():
    return 100

class algoritmo:
    def __init__(self, mapa, origen, destino, seleccion_mapa, dist_seguridad, muestreo_global, muestreo_restringido):
        self.roadmap = Roadmap(mapa.canvas, mapa) # Creamos el roadmap
        self.busqueda = Algoritmo_busqueda(3) # Algoritmo de búsqueda tipo A*
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.iteracion = 1

    def conectar_a_grafo(self, p):
        distancia_min = float("inf")
        punto_conexion = None
        segmento_conexion = None

        # Recorremos todas las aristas del roadmap
        for v1 in self.roadmap.vertices:
            for v2 in self.roadmap.vertices[v1]:
                d, pi = mapas.distancia_punto_segmento(p, v1, v2)
                if d < distancia_min and self.mapa.comprobar_segmento(p, pi):
                    distancia_min = d
                    punto_conexion = pi
                    segmento_conexion = (v1, v2)

        if punto_conexion:
            # Añadir el nuevo punto al roadmap y conectarlo a los extremos del segmento
            self.roadmap.añadir_arista(punto_conexion, segmento_conexion[0])
            self.roadmap.añadir_arista(punto_conexion, segmento_conexion[1])
            self.roadmap.dibujar_arista(punto_conexion, segmento_conexion[0])
            self.roadmap.dibujar_arista(punto_conexion, segmento_conexion[1])
            self.roadmap.dibujar_nodos(punto_conexion)
            return punto_conexion
        return None

    def iteraciones(self, max_iter):
        for i in range(max_iter):
            if not self.iteracion % 100: # Cada 100 iteraciones intentamos encontrar un camino
                origen_conectado = self.conectar_a_grafo(self.origen)
                destino_conectado = self.conectar_a_grafo(self.destino)
                if origen_conectado and destino_conectado:
                    camino = self.busqueda.resolver(origen_conectado, destino_conectado, self.roadmap)
                    if camino:
                        camino.insert(0, self.origen)
                        camino.append(self.destino)
                        self.roadmap.dibujar_camino(camino)
                        self.mapa.dibujar_origen_destino(self.origen, self.destino)
                        print("ÉXITO en la iteración: ", self.iteracion)
                        print("Longitud de la ruta: ", self.roadmap.long_camino)
                        print("Distancia mínima a un obstáculo: ", self.roadmap.dist_minima)
                        print("Número de nodos del árbol: ", len(self.roadmap.vertices))
                        return True, math.inf, math.inf, len(self.roadmap.vertices)
            else:
                qn = self.mapa.muestra_aleatoria() # Generamos un punto aleatorio
                self.roadmap.añadir_vertice(qn, self.mapa) # Añadimos el punto aleatorio al roadmap
            self.iteracion += 1 # Nueva iteracion realizada
            print("Iteración: ", self.iteracion)
        return False, math.inf, math.inf, len(self.roadmap.vertices)

class Nodo:
    # CONSTRUCTOR
    def __init__(self, estado, g, h, padre):
        self.estado = estado # Posición (x,y) del nodo
        self.padre = padre # Para saber de donde viene
        self.g = g # Coste de ir desde el oriden hasta el nodo
        self.f = g + h # Coste para ordenar los nodos

    # FUNCIÓN QUE DEVUELVE TRUE SI 2 NODOS TIENEN EL MISMO ESTADO
    def __eq__(self, other):
        return self.estado == other.estado
    
    # FUNCIÓN QUE DEVUELVE TRUE SI 2 NODOS TIENEN DISTINTO ESTADO
    def __ne__(self, other):
        return self.estado != other.estado

class Algoritmo_busqueda:
    # CONSTRUCTOR
    def __init__(self, tipo):
        if tipo == 1: # Dijkstra
            self.peso_g = 1
            self.peso_h = 0
        elif tipo == 2: # Best First
            self.peso_g = 0
            self.peso_h = 1
        else: # A*
            self.peso_g = 1
            self.peso_h = 1

    # FUNCIÓN QUE DEVUELVE LOS VECINOS DE UN NODO DADO
    def obtener_vecinos(self, estado, destino, roadmap):
        v = roadmap.obtener_vecinos(estado.estado)
        if estado.padre and estado.padre.estado in v:
            v.remove(estado.padre.estado)
        vecinos = [Nodo(n, self.peso_g*(estado.g + mapas.distancia_entre_puntos(estado.estado, n)), self.peso_h*mapas.distancia_entre_puntos(n, destino), estado) for n in v]
        return vecinos

    # FUNCIÓN QUE DEVUELVE EL CAMINO UNA VEZ ENCONTRADO
    def obtener_camino(self, estado):
        camino = []
        while estado:
            camino.append(estado.estado)
            estado = estado.padre
        return camino[::-1] # Invierte el camino porque empieza por destino

    # FUNCIÓN QUE EJECUTA EL ALGORITMO DE PLANIFICACIÓN
    def resolver(self, origen, destino, roadmap):
        nodos_visitados = []
        nodos_descubiertos = [Nodo(origen, 0, mapas.distancia_entre_puntos(origen, destino), None)]

        while nodos_descubiertos:           
            # El primer paso es seleccionar el nodo con menor coste
            nodos_descubiertos.sort(key=lambda o: o.f) # Ordenamos los nodos de menor a mayor coste
            nodo_actual = nodos_descubiertos.pop(0) # Cogemos el nodo con menor coste
            nodos_visitados.append(nodo_actual) # Lo eliminamos de nodos descubiertos

            # Si el nodo seleccionado es el destino, hemos terminado
            if nodo_actual.estado == destino:
                return self.obtener_camino(nodo_actual)
            
            # Obtenemos los nodos adyacentes y, si procede, lo añadimos a descubiertos
            for vecino in self.obtener_vecinos(nodo_actual, destino, roadmap):
                if vecino in nodos_visitados: # Si el vecino ya ha sido analizado, pasamos al siguiente vecino
                    continue
                if vecino in nodos_descubiertos: # Si el vecino ya ha sido descubierto, actualizo la info si este es mejor
                    ind = nodos_descubiertos.index(vecino)
                    if nodos_descubiertos[ind].f > vecino.f:
                        nodos_descubiertos[ind] = vecino
                else: # Si el vecino no esta descubiertos, lo metemos
                    nodos_descubiertos.append(vecino)
        return None

class Roadmap:
    # CONSTRUCTOR
    def __init__(self, canvas, mapa):
        self.mapa = mapa
        self.vertices = {} # Diccionario donde cada vértice almacena aquellos a los que está conectado
        self.canvas = canvas # Canvas donde se dibuja el roadmap
        self.long_camino = math.inf # Para conocer la longitud de un camino una vez obtenido
        self.dist_minima = math.inf # Distancia mínima a un obstáculo en los nodos del camino
    
    # FUNCION PARA DIBUJAR LAS ARISTAS DEL ROADMAP
    def dibujar_arista(self, p1, p2):
        auxiliar.pygame.draw.line(self.canvas, auxiliar.color_arista, p1[:2], p2[:2], auxiliar.grosor_arista)

    # FUNCION PARA DIBUJAR LOS NODOS DEL ROADMAP
    def dibujar_nodos(self, p2):
        auxiliar.pygame.draw.circle(self.canvas, auxiliar.color_nodo, p2[:2], auxiliar.radio_nodo, auxiliar.radio_nodo)

    # FUNCION PARA DIBUJAR EL CAMINO UNA VEZ ENCONTRADO
    def dibujar_camino(self, camino):
        self.dist_minima = math.inf
        self.long_camino = 0
        pi = camino[0]
        for pf in camino[1:]:
            self.long_camino += mapas.distancia_entre_puntos(pi, pf)
            self.dist_minima = min(self.dist_minima, self.mapa.distancia_minima_a_obstaculos(pi), self.mapa.distancia_minima_a_obstaculos(pf))
            auxiliar.pygame.draw.line(self.canvas, auxiliar.rojo, pi[:2], pf[:2], auxiliar.grosor_camino)
            pi = pf
        auxiliar.pygame.display.update()

    # FUNCIÓN QUE SIRVE PARA AÑADIR UN PUNTO AL ROADMAP
    def añadir_vertice(self, qn, mapa):
        if qn in self.vertices: # Si el punto ya pertenece al roadmap, no hacemos nada
            return
        if not mapa.comprobar_punto(qn): # Si el punto no es válido por chocar con algún obstáculo, no hacemos nada
            return
        self.vertices[qn] = []
        self.dibujar_nodos(qn)
        vecinos = self.obtener_nodos_cercanos(qn, radio_busqueda())[:5] # Como mucho, cogemos 5 vecinos
        for vecino in vecinos:
            if mapa.comprobar_segmento(qn, vecino[0]): # Si la arista entre el vecino y p no choca con ningún obstáculo
                self.añadir_arista(qn, vecino[0])
                self.dibujar_arista(qn, vecino[0])
        return

    # FUNCIÓN PARA AÑADIR UNA ARISTA AL ROADMAP
    def añadir_arista(self, v1,v2):
        if v1 not in self.vertices: self.vertices[v1] = []
        if v2 not in self.vertices: self.vertices[v2] = []
        self.vertices[v1].append(v2)
        self.vertices[v2].append(v1)

    # ESTA FUNCION SIRVE PARA ENCONTRAR NODOS CERCANOS A UN PUNTO p
    def obtener_nodos_cercanos(self, p, r):
        nodos = [] # En este vector se almacenaran los nodos cercanos
        for n in self.vertices:
            d = mapas.distancia_entre_puntos(p,n) # Distancia de p al nodo n
            if d < r: # Si el nodo esta centro de un circulo de radio r
                # Cada nodo se guarda en el formato (nodo, distancia)
                nodos.append((n, d))
        nodos.sort(key=lambda x:x[1]) # Ordenamos los nodos en menor a mayor distancia
        return nodos

    # FUNCIÓN QUE DEVUELVE TODOS LOS NODOS CONECTADOS A TRAVÉS DEL ROADMAP A UNO DADO
    def obtener_vecinos(self, v):
        return self.vertices[v]