# EN ESTE ARCHIVO, SE DEFINEN UNA SERIE DE CONSTANTES QUE SE USARAN EN OTROS .py
import pygame

# VARIABLES PARA REPRESENTACION GRAFICA
gris = (70,70,70)
azul = (0,0,255)
verde = (0,255,0)
rojo = (255,0,0)
amarillo = (255,255,0)
blanco = (255,255,255)
radio_nodo = 2 # Radio de los puntos inicial y final para su representacion grafica
grosor_arista = 1 # Ancho de la arista del árbol
grosor_camino = 4 # Grosor con el que se pinta el camino encontrado
color_arista = azul # Color de la arista
color_nodo = azul # Color del nodo

# MAPA
tamaño_mapa = (1000, 600) # Tamaño del mapa
numero_objetos = 200 # Numero de obstaculos en caso de mapa aleatorio
minimo_tamaño_obstaculo = 10 # Tamaño minimo de cada obstaculo
varianza_obstaculos = 50 # Tamaño maximo de cada obstaculo
paso_planificador = 3 # Cada cuanta distancia toma una muestra en un segmento para detectar colisiones

def pygame_events(event_list = []):
    pygame.display.update()
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            return False
        for e in event_list:
            if evento.type == e:
                return  e
    return True

# ESTA FUNCION ESPERA A QUE EL USUARIO PRESIONE UNA TECLA
def pygame_esperar_tecla():
    while(True):
        for evento in pygame.event.get():
            if evento.type == pygame.KEYDOWN:
                return evento.key
            if evento.type == pygame.QUIT:
                return pygame.QUIT