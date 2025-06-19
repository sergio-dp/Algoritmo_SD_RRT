import auxiliar
import copy
import mapas
import math
import os
import random
from collections import deque

# FUNCIÓN ESPECÍFICA PARA EL RRT* QUE CALCULA EL RADIO DE BÚSQUEDA DE NODOS VECINOS
def radio_optimo(n, tamaño_mapa = auxiliar.tamaño_mapa[0], gamma = 0.7):
    return gamma*tamaño_mapa*(math.log(n+1)/(n+1))**0.5

# FUNCIÓN QUE INCREMENTA EL COSTE DE LOS NODOS CERCA DE LOS OBSTÁCULOS
def incremento_coste(coste, distancia, distancia_limite):
    if distancia >= distancia_limite: # Si la distancia al obstáculo es mayor que la distancia límite, no incrementamos el coste
        return coste
    return (100*(distancia_limite - distancia) + 1) * coste

class elipse():
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.c_min = mapas.distancia_entre_puntos(p1,p2)
        self.centro = (0.5*(p2[0]+p1[0]), 0.5*(p2[1]+p1[1]))
        self.angulo = math.atan2(p2[1]- p1[1], p2[0] - p1[0])
        self.R = ((math.cos(self.angulo), -math.sin(self.angulo)),(math.sin(self.angulo), math.cos(self.angulo)))

class algoritmo:
    def __init__(self, mapa, origen, destino, seleccion_mapa, dist_seguridad, muestreo_global, muestreo_restringido):
        self.arbol = Arbol(origen, destino, dist_seguridad, mapa.canvas, seleccion_mapa, mapa)
        self.nodos_importantes = copy.deepcopy(self.arbol.nodos_importantes_anteriores)
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.iteracion = 1 # Vector donde cada nodo almacena a su padre
        self.elipses = {origen: {destino: elipse(origen, destino)}}
        self.elipse_actual = None
        self.muestreo_global = muestreo_global
        self.muestreo_restringido = muestreo_restringido

    def obtener_elipse(self, p1, p2):
        if p1 in self.elipses:
            if p2 in self.elipses[p1]:
                return self.elipses[p1][p2]
        el = elipse(p1,p2)
        self.elipses[p1] = {p2:el}
        return el

    # FUNCION QUE CAMBIA LA ELIPSE DEL INFORMED - RRT*
    def tomar_muestra(self):
        if self.arbol.long_camino == math.inf: # En el caso de no haber encontrado aún camino, muestreamos en todo el espacio
            self.elipse_actual =  self.obtener_elipse(self.origen, self.destino)
            return self.arbol.mapa.muestra_aleatoria(), math.inf, math.inf
        
        if random.random() < self.muestreo_global:
            return self.arbol.mapa.muestra_aleatoria(), math.inf, math.inf
        
        elipse_total =  self.obtener_elipse(self.origen, self.destino)       
        if random.random() < self.muestreo_restringido:
            q1, q2 = self.arbol.obtener_sub_camino()
            self.elipse_actual = self.obtener_elipse(q1 , q2)
            c_max = self.arbol.coste_nodo[q2] - self.arbol.coste_nodo[q1]
        else:
            self.elipse_actual, c_max, q1, q2 = elipse_total, self.arbol.long_camino, self.origen, self.destino
        
        if c_max - self.elipse_actual.c_min <=0:
            self.elipse_actual, c_max, q1, q2 = elipse_total, self.arbol.long_camino, self.origen, self.destino

        alpha, r1, r2 = self.arbol.mapa.muestra_restringida(q1, q2, c_max, self.elipse_actual.c_min)
        return alpha, r1, r2

    def iteraciones(self, max_iter):
        arbol = self.arbol
        destino = self.destino
        for i in range(max_iter):
            alpha, r1, r2 = self.tomar_muestra()
            nodo_importante = False
            if not self.iteracion % 100 and not destino in arbol.padres:
                alpha = destino
            elif not self.iteracion % 10 and self.nodos_importantes:
                nodo_importante = True
                while self.nodos_importantes:
                    candidato = self.nodos_importantes[0]
                    if candidato not in arbol.padres:
                        alpha = candidato
                        self.nodos_importantes = self.nodos_importantes[1:] + [self.nodos_importantes[0]]
                        break
                    else:
                        self.nodos_importantes.remove(candidato)
            qn, arista = arbol.punto_mas_cercano(alpha) # Punto y arista mas cercanos del arbol a alpha
            qs, choque = self.arbol.mapa.punto_de_parada(qn, alpha) # Punto de parada al trazar la arista desde qn hasta alpha
            if qs != qn or qs == destino: # Si la arista tiene longitud
                arbol.cablear_arbol(qs, qn, arista, choque, r1, r2, self.elipse_actual.centro, self.elipse_actual.angulo)
                if nodo_importante and qs == alpha:
                    self.nodos_importantes.remove(candidato)
                    conexiones_hechas = [alpha]
                    while self.nodos_importantes:
                        # Vemos desde que nodos importantes podemos conectar al nodo importante actual
                        visto_desde = {}
                        nodos_conectados = False
                        for p1 in conexiones_hechas:
                            for p2 in self.nodos_importantes:
                                if self.arbol.mapa.comprobar_segmento(p1, p2):
                                    if p2 not in visto_desde: visto_desde[p2] = []
                                    visto_desde[p2].append(p1)

                        # Conectados el nodo importante de la forma más óptima
                        for p2, p1_list in visto_desde.items():
                            nodo_origen = None
                            dist_min = math.inf
                            for nodo in p1_list:
                                dist = mapas.distancia_entre_puntos(nodo, p2) + self.arbol.coste_nodo[nodo]
                                if dist < dist_min:
                                    dist_min = dist
                                    nodo_origen = nodo
                            arbol.cablear_arbol(p2, nodo_origen, None, False, math.inf, math.inf, self.elipse_actual.centro, self.elipse_actual.angulo)
                            conexiones_hechas.append(p2)
                            self.nodos_importantes.remove(p2)
                            nodos_conectados = True
                        if not nodos_conectados: # Si no hemos conectado ningún nodo importante, salimos del bucle
                            break
            self.iteracion += 1 # Nueva iteracion realizada
            if destino in arbol.padres: # El objetivo pertenece al arbol
                arbol.dibujar_camino() # Dibujamos el mejor camino encontrado 
        return False, arbol.long_camino, arbol.dist_minima, len(arbol.padres)+1

class Arbol:
    # CONSTRUCTOR
    def __init__(self, origen, destino,  dist_seguridad, canvas, seleccion_mapa, mapa):
        # VARIABLES TÍPICAS DE CUALQUIER ALGORITMO RRT
        self.mapa = mapa
        self.origen = origen # Nodo origen
        self.destino = destino # Nodo objetivo
        self.long_camino = math.inf # Para conocer la longitud de un camino una vez obtenido
        self.padres = {} # Vector donde cada nodo almacena a su padre
        self.canvas = canvas # Canvas donde se dibuja el árbol
        self.coste_nodo = {origen:0} # Vector donde cada nodo almacena el coste acumulado de llegar desde el origen
        self.coste_arista = {} # Vector donde cada nodo almacena el coste de llegar desde el padre (distancia arista)
        
        # VARIABLES RELACIONADAS CON LOS OBSTÁCULOS
        self.dist_seguridad = dist_seguridad # Distancia mínima a la pared
        self.chocado = {} # Vector que sirve para saber que nodos han chocado contra un obstáculo
        self.dist_obstaculo = {origen:math.inf} # Vector que sirve para saber que nodos están cerca de los obstáculos
        self.dist_minima = math.inf # Distancia mínima a un obstáculo en los nodos del camino
        
        # Leemos los nodos de choque de planificaciones anteriores
        self.seleccion_mapa = seleccion_mapa
        if self.seleccion_mapa != 0:
            subcarpeta = "ficheros"
            nombre_archivo = f"obstaculos_{self.seleccion_mapa}.txt"
            self.archivo_nodos_choque = os.path.join(subcarpeta, nombre_archivo)
            os.makedirs(subcarpeta, exist_ok=True)
            try:
                with open(self.archivo_nodos_choque, "r") as archivo:
                    for linea in archivo:
                        x, y = map(int, linea.strip().split(","))
                        nodo = x, y
                        if self.mapa.comprobar_punto(nodo):
                            self.chocado[nodo] = True
            except FileNotFoundError:
                with open(self.archivo_nodos_choque, "w") as f:
                    pass  # Esto crea el archivo vacío

        # Leemos los nodos destacados de planificaciones anteriores
        self.nodos_importantes_anteriores = []
        if self.seleccion_mapa != 0:
            nombre_archivo = f"nodos_destacados_{self.seleccion_mapa}.txt"
            self.archivo_nodos_destacados = os.path.join(subcarpeta, nombre_archivo)
            try:
                with open(self.archivo_nodos_destacados, "r") as archivo:
                    for linea in archivo:
                        x, y = map(int, linea.strip().split(","))
                        nodo = x, y
                        if self.mapa.comprobar_punto(nodo):
                            self.nodos_importantes_anteriores.append(nodo)

                    # Ordenar por distancia al origen
                    self.nodos_importantes_anteriores.sort(key=lambda nodo: mapas.distancia_entre_puntos(nodo, self.origen))
            except FileNotFoundError:
                with open(self.archivo_nodos_choque, "w") as f:
                    pass  # Esto crea el archivo vacío

    # FUNCIÓN DE DIBUJO GLOBAL
    def dibujar(self, r1 = math.inf, r2 = math.inf, centro = None, angulo = None):
        for p2 in self.padres: # Dibuja las aristas del arbol
            self.dibujar_arista(p2)
            self.dibujar_nodos(p2)
        for p2 in self.chocado: # Dibuja los obstáculos detectados en planificaciones anteriores
            if not p2 in self.padres:
                self.dibujar_nodos(p2)
        for p2 in self.nodos_importantes_anteriores: # Dibuja los nodos destacados de planificaciones anteriores
            self.dibujar_nodos(p2)
        if self.destino in self.padres: # Si existe camino entre el origen y el objetivo, lo señalamos de rojo
            self.dibujar_camino()
            self.dibujar_elipse(r1, r2, centro, angulo)

    # FUNCIÓN PARA DIBUJAR LAS ARISTAS DEL ÁRBOL
    def dibujar_arista(self, p2):
        # El color de la arista dependerá de la localización de los extremos
        if self.padres[p2][:2] == self.origen[:2] or (self.dist_obstaculo[self.padres[p2]] >= self.dist_seguridad and self.dist_obstaculo[p2] >= self.dist_seguridad):
            color = auxiliar.azul
        elif self.padres[p2] in self.chocado and p2 in self.chocado:
            color = auxiliar.rojo
        else:
            r1, g1, b1 = self.calcular_color(self.dist_obstaculo[p2])
            r2, g2, b2 = self.calcular_color(self.dist_obstaculo[self.padres[p2]])
            color = (int((r1+r2)*0.5), int((g1+g2)*0.5), int((b1+b2)*0.5))
        auxiliar.pygame.draw.line(self.canvas, color, self.padres[p2][:2], p2[:2], auxiliar.grosor_arista)

    # FUNCIÓN PARA DIBUJAR LOS NODOS DEL ÁRBOL
    def dibujar_nodos(self, p2):
        if p2 in self.nodos_importantes_anteriores and p2 not in self.padres:
            color, radio = auxiliar.amarillo, 3*auxiliar.radio_nodo
        elif p2 in self.nodos_importantes_anteriores and p2 in self.padres:
            color, radio = auxiliar.gris, 3*auxiliar.radio_nodo
        elif p2 in self.chocado: # Los nodos que han chocado con un obstaculo, los pintamos de rojo
            color, radio = auxiliar.rojo, auxiliar.radio_nodo
        elif self.dist_obstaculo[p2] < self.dist_seguridad: # Nodo cerca de obstáculo
            color, radio = self.calcular_color(self.dist_obstaculo[p2]), auxiliar.radio_nodo
        else:
            color, radio = auxiliar.azul, auxiliar.radio_nodo
        auxiliar.pygame.draw.circle(self.canvas, color, p2[:2], radio, radio)

    # FUNCIÓN QUE CALCULA EL COLOR SEGÚN LA DISTANCIA AL OBSTÁCULO
    def calcular_color(self, distancia):
        if distancia > self.dist_seguridad:
            distancia = self.dist_seguridad
        r = int(255-(255*(distancia / self.dist_seguridad)))
        g = int(255*(distancia / self.dist_seguridad))
        b = int(0)
        return (r, g, b)

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

    # ESTA FUNCION SIRVE PARA DIBUJAR LA ELIPSE DEL INFORMED - RRT*
    def dibujar_elipse(self, r1, r2, centro, angulo):
        if r1 != math.inf and r2 != math.inf: # Anti - errores
            # Primero dibujamos la elipse
            rectangulo = auxiliar.pygame.Rect(centro[0]-r1, centro[1]-r2,2*r1,2*r2)
            elipse = auxiliar.pygame.Surface(rectangulo.size, auxiliar.pygame.SRCALPHA)
            auxiliar.pygame.draw.ellipse(elipse, auxiliar.verde, (0, 0, 2*r1, 2*r2), 4*auxiliar.grosor_arista)
            elipse_rotada = auxiliar.pygame.transform.rotate(elipse, -math.degrees(angulo)) # Rotar la elipse
            rectangulo_rotado = elipse_rotada.get_rect(center = rectangulo.center) # Obtener el rectángulo de la superficie rotada
            self.canvas.blit(elipse_rotada, rectangulo_rotado) # Dibujar la elipse en el canvas

            # Finalmente, el punto central de la elipse
            auxiliar.pygame.draw.circle(self.canvas, auxiliar.verde, centro, 3*auxiliar.radio_nodo, 3*auxiliar.radio_nodo)

    # FUNCIÓN PARA OBTENER EL CAMINO ENTRE ORIGEN Y DESTINO
    def obtener_camino(self):
        camino = deque()
        nodo = self.destino
        while nodo in self.padres:
            camino.appendleft(nodo)
            nodo = self.padres[nodo]
        camino.appendleft(self.origen)
        return list(camino)
    
    # FUNCIÓN PARA OBTENER UN SUBCAMINO ENTRE 2 PUNTOS CUALESQUIERA DE LA RUTA
    def obtener_sub_camino(self):
        camino = self.obtener_camino()
        while(True):
            i1, i2 = sorted(random.sample(range(len(camino)), 2))
            if i1 < i2:
                return camino[i1] , camino[i2]

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
    def cablear_arbol(self, qs, qn, arista, choque, r1, r2, elipse_centro, elipse_angulo):
        repintar_mapa = False
        radio_busqueda = radio_optimo(len(self.padres))
        vecinos = self.obtener_vecinos(qs, radio_busqueda, choque)
        distancia_minima = mapas.distancia_entre_puntos(qs,qn)
        if arista:
            d_min = math.inf
            for obs in self.chocado:
                d = mapas.distancia_entre_puntos(qn, obs)
                if d < d_min:
                    d_min = d
            coste_minimo = distancia_minima + self.coste_nodo[self.padres[arista]] + mapas.distancia_entre_puntos(self.padres[arista],qn)
            coste_minimo = incremento_coste(coste_minimo, d_min, self.dist_seguridad)
        else:
            coste_minimo = distancia_minima + self.coste_nodo[qn]
            coste_minimo = incremento_coste(coste_minimo, self.dist_obstaculo[qn], self.dist_seguridad)

        # Primer paso, vemos el punto mas optimo al que conectar el punto de parada
        for vecino in vecinos:
            if incremento_coste(vecino[3], self.dist_obstaculo[vecino[0]], self.dist_seguridad) < coste_minimo and self.mapa.comprobar_segmento(vecino[0],qs):
                self.añadir_arista(vecino[0], qs, None)
                vecinos.remove(vecino)
                break
        else: # El coste mas optimo es conectar qs al arbol a traves de qn
            self.añadir_arista(qn, qs, arista)

        # Segundo paso, intentamos reconectar a traves de qs para reducir el coste
        coste_minimo = self.coste_nodo[qs]
        for vecino in vecinos:
            coste_original = incremento_coste(vecino[2], self.dist_obstaculo[self.padres[vecino[0]]], self.dist_seguridad)
            coste_nuevo = incremento_coste(coste_minimo + vecino[1], self.dist_obstaculo[qs], self.dist_seguridad)
            if coste_nuevo < coste_original and self.mapa.comprobar_segmento(vecino[0],qs):
                self.cambiar_padre(vecino[0],qs) # Cambiamos el padre
                repintar_mapa = True
        
        # Una vez recableado el árbol, recorremos el camino y "segmentamos" las aristas más largas
        if self.destino in self.padres:
            camino = self.obtener_camino()
            for i in range(1, len(camino)):
                nodo = camino[i]
                padre = self.padres[nodo]
                if self.coste_arista[nodo] > radio_busqueda:
                    nuevo_nodo = (int((nodo[0] + padre[0])*0.5), int((nodo[1] + padre[1])*0.5))
                    if not nuevo_nodo in self.padres:
                        self.padres[nuevo_nodo], self.padres[nodo] = padre, nuevo_nodo
                        self.calcular_coste(nuevo_nodo)
                        repintar_mapa = True

                        # Calculamos su distancia a obstaculos
                        d_min = math.inf
                        for obs in self.chocado:
                            d = mapas.distancia_entre_puntos(nuevo_nodo, obs)
                            if d < d_min:
                                d_min = d
                        self.dist_obstaculo[nuevo_nodo] = d_min
                        self.calcular_coste(nuevo_nodo)

                        # En el caso de añadir un punto intermedio, intentamos recablear el segmento. Empezamos por camino[i]
                        if nodo in self.chocado:
                            vecinos = self.obtener_vecinos(nodo, radio_busqueda, True)
                        else:
                            vecinos = self.obtener_vecinos(nodo, radio_busqueda, False)
                        for vecino in vecinos:
                            coste_original = incremento_coste(self.coste_nodo[nodo], self.dist_obstaculo[self.padres[nodo]], self.dist_seguridad)
                            coste_nuevo = incremento_coste(vecino[3], self.dist_obstaculo[vecino[0]], self.dist_seguridad)
                            if coste_nuevo < coste_original and self.mapa.comprobar_segmento(nodo, vecino[0]):
                                self.cambiar_padre(nodo, vecino[0])

                        # Reconectamos el punto intermedio
                        vecinos = self.obtener_vecinos(nuevo_nodo, radio_busqueda, False)
                        for vecino in vecinos:
                            coste_original = incremento_coste(self.coste_nodo[nuevo_nodo], self.dist_obstaculo[padre], self.dist_seguridad)
                            coste_nuevo = incremento_coste(vecino[3], self.dist_obstaculo[vecino[0]], self.dist_seguridad)
                            if coste_nuevo < coste_original and self.mapa.comprobar_segmento(nuevo_nodo, vecino[0]):
                                self.cambiar_padre(nuevo_nodo, vecino[0])

        if repintar_mapa:
            self.mapa.dibujar()
            if r1 != math.inf and r2 != math.inf:
                self.dibujar(r1, r2, elipse_centro, elipse_angulo)
            else:
                self.dibujar()
            self.mapa.dibujar_origen_destino(self.origen, self.destino)

    # ESTA FUNCION SIRVE PARA ENCONTRAR NODOS CERCANOS A UN PUNTO p
    def obtener_vecinos(self, qs, r, choque):
        # Vemos si el nodo ha chocado
        if choque == True:
            self.chocado[qs], self.dist_obstaculo[qs] = True, 0
        else:
            d_min = math.inf
            for n in self.chocado:
                d = mapas.distancia_entre_puntos(qs, n)
                if d < d_min:
                    d_min = d
            self.dist_obstaculo[qs] = d_min

        vecinos = [] # En este vector se almacenarán los nodos cercanos
        for n in self.padres:
            d = mapas.distancia_entre_puntos(qs,n) # Distancia de p al nodo n
            if d < r: # Si el nodo está dentro del radio de búsqueda
                vecinos.append((n, d, self.coste_nodo[n], d + self.coste_nodo[n])) # Cada nodo se guarda en el formato (nodo, distancia, coste origen-nodo n, coste origen-punto p)
            if choque == True and d < self.dist_obstaculo[n]:
                self.dist_obstaculo[n] = d
        vecinos.sort(key=lambda x:x[3])
        return vecinos

    # FUNCIÓN QUE SIRVE PARA AÑADIR UNA ARISTA AL ARBOL
    def añadir_arista(self, padre, hijo, arista): # p1 es el punto de conexión, p2 el punto a añadir y p3 la arista
        if hijo in self.padres: # Si p2 ya pertenece al arbol, no hacemos nada
            return False

        # Cambiamos los padres según el caso
        if arista and arista in self.padres and not padre in self.padres: # Este caso se da si vamos a conectar p2 al punto medio p1 de la arista p3 - padre_p3
            self.padres[padre], self.padres[hijo], self.padres[arista] = self.padres[arista], padre , padre
            self.dist_obstaculo[padre] = math.inf
        else: # Este caso se da si vamos a conectar p2 a un nodo del arbol
            self.padres[hijo] = padre

        # Actualizamos todos los costes
        if arista:
            self.calcular_coste(padre)
        self.calcular_coste(hijo)
        if arista:
            self.calcular_coste(arista)
    
    # ESTA FUNCIÓN SIRVE PARA CALCULAR LOS COSTES ASOCIADOS A UN NODO NUEVO
    def calcular_coste(self, p):
        if not p in self.padres: # Si el nodo no tiene padre, es el origen
            return
        self.coste_arista[p] = mapas.distancia_entre_puntos(self.padres[p],p) # Distancia entre un nodo y su padre
        self.coste_nodo[p] = self.coste_arista[p] + self.coste_nodo[self.padres[p]]

    # ESTA FUNCIÓN SIRVE PARA CAMBIAR EL PADRE DE UN NODO
    def cambiar_padre(self, p, nuevo_padre):
        if not p in self.padres: # Si el nodo no esta en el arbol, no hace nada
            return
        antiguo_padre = self.padres[p] # Guardamos el padre antiguo
        self.padres[p] = nuevo_padre # Cambia el padre
        if self.detección_bucles() == True: # Evita bucles infinitos
            self.padres[p] = antiguo_padre
            return

        # Actualiza el coste
        self.actualizar_coste(p)

    # ESTA FUNCIÓN SIRVE PARA DETECTAR BUCLES
    def detección_bucles(self):
        visitado_global = set()
        for nodo in self.padres:
            visitado_local = set()
            actual = nodo
            while actual in self.padres:
                if actual in visitado_local:
                    return True  # Se detectó un ciclo
                if actual in visitado_global:
                    break  # Ya revisamos este camino antes, sin ciclo
                visitado_local.add(actual)
                actual = self.padres[actual]
            visitado_global.update(visitado_local)
        return False  # No se encontraron ciclos

    # ESTA FUNCIÓN SIRVE PARA ACTUALIZAR LOS COSTES DEL ÁRBOL AL RECABLEAR EL ARBOL
    def actualizar_coste(self, p):
        if not p in self.padres: # Si el nodo no esta en el arbol, no hace nada
            return
        self.calcular_coste(p)

        # Cogemos los nodos que tienen a p como padre
        hijos = [nodo for nodo, padre in self.padres.items() if padre == p]
        for hijo in hijos:
            self.actualizar_coste(hijo)

    # ESTA FUNCIÓN SIRVE PARA GUARDAR EN UN .txt TODOS LOS NODOS QUE HAN CHOCADO CON OBSTÁCULOS
    def guardar_nodos(self):
        if self.seleccion_mapa == 0:
            return
        # Guardamos los nodos destacados
        # Primero, metemos los nodos importantes anteriores que no pertenecen al árbol actual
        for p in self.nodos_importantes_anteriores:
            if p not in self.padres:
                self.dist_obstaculo[p] = min(mapas.distancia_entre_puntos(p, n) for n in self.chocado)

        # Ordenamos los nodos por orden de hijos de mayor a menor
        nodos_ordenados = sorted((n for n in self.dist_obstaculo if n != self.origen and n != self.destino), key=lambda n: self.dist_obstaculo[n], reverse=True)

        # De más a menos distancia, filtramos usando el concepto de visibilidad
        nodos_guardianes = []
        nodos_chocados = []
        umbral_seguridad = self.dist_seguridad / 4
        nodos_chocados_reales = [n for n, chocado in self.chocado.items() if chocado]
        for nodo in nodos_ordenados + nodos_chocados_reales:
            es_chocado = nodo in nodos_chocados_reales
            if es_chocado:
                if any(mapas.distancia_entre_puntos(nodo, g) < umbral_seguridad and(abs(nodo[0] - g[0]) < 2 or abs(nodo[1] - g[1]) < 2) for g in nodos_chocados):
                    continue
                nodos_chocados.append(nodo)
            if not any(self.mapa.comprobar_segmento(nodo, g) for g in nodos_guardianes):
                nodos_guardianes.append(nodo)

        # En caso de no ser nodo guardian, lo intentamos usar para mejorar la conectividad como nodo de conexión
        nodos_de_conexion = {}
        if len(nodos_guardianes) > 1:
            for nodo in nodos_ordenados:
                if nodo in nodos_guardianes:
                    continue
                for i, v1 in enumerate(nodos_guardianes):
                    for v2 in nodos_guardianes[i+1:]:
                        if v1 != v2 and self.mapa.comprobar_segmento(nodo, v1) and self.mapa.comprobar_segmento(nodo, v2):
                            if any(v1 in ps and v2 in ps for ps in nodos_de_conexion.values()):
                                continue
                            nodos_de_conexion[nodo] = [v1, v2]

        # Guardamos los nodos importantes
        nodos_a_guardar = set(nodos_guardianes).union(nodos_de_conexion.keys())
        with open(self.archivo_nodos_destacados, "w") as archivo:
            for nodo in nodos_a_guardar:
                x, y = nodo
                archivo.write(f"{int(x)},{int(y)}\n")
        with open(self.archivo_nodos_choque, "w") as archivo:
            for nodo in nodos_chocados:
                archivo.write(f"{nodo[0]},{nodo[1]}\n")
        print("NODOS GUARDADOS")