import tkinter as tk
from enum import Enum
from threading import *
import auxiliar
import mapas
import PRM
import PRM_perezoso
import PRM_visibility
import RRT
import RRT_connect
import RRT_estrella
import RRT_informed
import RRT_N_informed
import SD_RRT_estrella

class Estado(Enum):
    PLAY = 1
    PAUSE = 2
    STOP = 3

context = {'event':False,
           'key':None,
           'planner': None,
           'map': None,
           'state': Estado.STOP,
           'end': False,
           'dist_seg': 20,
           'muestreo_global': 0.5,
           'muestreo_restringido': 0.5}

planners = [("PRM", PRM.algoritmo),
            ("PRM perezoso", PRM_perezoso.algoritmo),
            ("Visibility PRM", PRM_visibility.algoritmo),
            ("RRT", RRT.algoritmo),
            ("RRT - Connect", RRT_connect.algoritmo),
            ("RRT*", RRT_estrella.algoritmo),
            ("Informed RRT*", RRT_informed.algoritmo),
            ("n-Informed RRT*", RRT_N_informed.algoritmo),
            ("SD - RRT*", SD_RRT_estrella.algoritmo)]

def play():
    global context
    if context['state'] != Estado.PAUSE:
        planner = planners[context['sel_planner'].get()][1]
        context['planner'] = planner(context['map'], context['init'], context['goal'], context['sel_mapa'], context['dist_seg'], context['muestreo_global'], context['muestreo_restringido'])
        context['map'].dibujar()
        context['map'].dibujar_origen_destino(context['init'], context['goal'])
    context['state'] = Estado.PLAY
    update_UI_states()

def pause():
    global context
    context['state'] = Estado.PAUSE
    update_UI_states()
    auxiliar.pygame.display.update()

def stop():
    global context
    context['state'] = Estado.STOP
    if planners[context['sel_planner'].get()][0] == "SD - RRT*":
        context['planner'].arbol.guardar_nodos()
        auxiliar.pygame.display.update()
    auxiliar.pygame.image.save(context['map'].canvas, "grafico_arbol.bmp")
    update_UI_states()

def set_map(i):
    if i == -1:
        context['init'] = (50, 50)
        context['goal'] = (850, 500)
        context['map'].crear_mapa_aleatorio(auxiliar.numero_objetos,[context['init'], context['goal']])
        context['sel_mapa'] = 0
    else:
        context['init'] = mapas.maps[i].init
        context['goal'] = mapas.maps[i].goal
        context['map'].cargar_mapa(mapas.maps[i].map,[context['init'], context['goal']])
        context['sel_mapa'] = i + 1
    context['map'].dibujar()
    context['map'].dibujar_origen_destino(context['init'],context['goal'])
    auxiliar.pygame.display.update()
    update_UI_states()

def frame_state(frame, state):
    for child in frame.winfo_children():
       child.configure(state=state)

def update_UI_states():
    global context
    bplay, bpause, bstop = context['play'], context['pause'], context['stop']
    frm_state = "disabled"
    if context['state'] == Estado.STOP:
        bpause["state"] = "disabled"
        bplay["state"] = "normal"
        bstop["state"] = "disabled"
        frm_state = "normal"
    elif context['state'] == Estado.PAUSE:
        bpause["state"] ="disabled"
        bplay["state"]="normal"
        bstop["state"]="normal"
    else:
        bpause["state"]="normal"
        bplay["state"]="disabled"
        bstop["state"]="normal"
    frame_state(context['frame_planners'],frm_state)
    frame_state(context['frame_map'],frm_state)

def on_close():
    global context
    context['end'] = True
    context['gui'].destroy()

def init_gui_window():
    global context
    context['gui'] = root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_close)

    # ELEGIMOS EL PLANIFICADOR  
    context['sel_planner'] = tk.IntVar()
    context['frame_planners'] = tk.Frame(root, borderwidth=2, relief=tk.GROOVE)
    for i in range(len(planners)):
        tk.Radiobutton(context['frame_planners'], text="(" + str(i+1) + ") " + str(planners[i][0]), padx=20, variable=context['sel_planner'], value=i).pack(anchor=tk.W)
    context['frame_planners'].pack(padx=5, pady=5)
    
    # PANEL DE CONTROL
    control_frame = tk.Frame(root)
    context['play'] = tk.Button(control_frame, text="PLAY", command=play)
    context['play'].pack(side=tk.LEFT, pady=15)
    context['pause'] = tk.Button(control_frame, text="PAUSE", command=pause)
    context['pause'].pack(side=tk.LEFT)
    context['stop'] = tk.Button(control_frame, text="STOP", command=stop)
    context['stop'].pack(side=tk.RIGHT)
    control_frame.pack(padx=5, pady=5)

    # ELEGIMOS EL MAPA
    context["frame_map"] = tk.Frame(root,borderwidth=2, relief=tk.GROOVE)
    tk.Button(context["frame_map"], text="Mapa Aleatorio", command=lambda: set_map(-1)).pack(fill=tk.X)
    for i in range(len(mapas.maps)):
        tk.Button(context["frame_map"], text=mapas.maps[i].description, command=lambda i=i: set_map(i)).pack(fill=tk.X)
    context["frame_map"].pack(padx=5, pady=5)
    update_UI_states()

def init_defaults():
    global context
    context['map'] = mapas.Mapa_base(*auxiliar.tamaño_mapa)
    context['init'] = mapas.maps[0].init
    context['goal'] = mapas.maps[0].goal
    context['sel_mapa'] = 1
    context['map'].cargar_mapa(mapas.maps[0].map, [context['init'], context['goal']])

def process_pygame_events():
    ev = auxiliar.pygame.event.get()
    for event in ev:
        if context['state'] == Estado.STOP:
            if event.type == auxiliar.pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    context['init'] = auxiliar.pygame.mouse.get_pos()
                    print("Nuevo origen:", context['init'])
                if event.button == 3:
                    context['goal'] = auxiliar.pygame.mouse.get_pos()
                    print("Nuevo destino:", context['goal'])
                context['map'].dibujar()
                context['map'].dibujar_origen_destino(context['init'], context['goal'])
                auxiliar.pygame.display.update()

def control_loop():
    global context
    while(not context['end']):
        process_pygame_events()
        if context['state'] == Estado.PLAY:
            auxiliar.pygame.display.update()
            final, _, _, _ = context['planner'].iteraciones(1)
            if final == True:
                stop()
 
# CÓDIGO PRINCIPAL
if __name__ == '__main__':
    auxiliar.pygame.init()
    init_gui_window()
    init_defaults()

    context['map'].dibujar()
    context['map'].dibujar_origen_destino(context['init'], context['goal'])
    auxiliar.pygame.display.update()

    Thread(target=control_loop).start() 
    context['gui'].mainloop()
    auxiliar.pygame.quit()