import cv2
import numpy as np
import math
import os
import subprocess
import socket
import threading
import csv      
import time     

RUTA_BASE = "Enter the base path here"
EJECUTABLE_HPPM = os.path.join(RUTA_BASE, "output\\HPPM.exe")
ARCHIVO_CALIBRACION = os.path.join(RUTA_BASE, "calibracion_data.npz")

hilos_ocupados = {1: False, 2: False, 3: False} 

CONFIG_ROBOTS = {
    1: {"ip": "192.168.137.158", "puerto": 5005, "id_meta": 11, "color": (0, 255, 255)}, # Amarillo
    2: {"ip": "192.168.137.42",  "puerto": 5005, "id_meta": 12, "color": (255, 100, 0)},  # Naranja
    3: {"ip": "192.168.137.XX",  "puerto": 5005, "id_meta": 13, "color": (0, 255, 0)}     # Verde 
}

RADIO_ROBOT = 0.03                 
RADIO_INFLADO = RADIO_ROBOT * 1.0 
MAGNITUD_K_ESTATICO = 0.001                 
K_ROBOT = 0.015                               

RADIO_INFLADO_ROBOT = RADIO_ROBOT * 0.5 

DISTANCIA_LLEGADA = 0.05 

FRECUENCIA_RUTAS = 12        
PASOS_MAXIMOS = "500"        
RADIO_S = "0.010000"         

M1_CUADRANTES_1_3 = "-4.000000"
M2_CUADRANTES_1_3 = "-1.000000"
M1_CUADRANTES_2_4 = "4.000000"
M2_CUADRANTES_2_4 = "1.000000"

diccionario_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MODO_AUTOMATICO = False 

rutas_memoria = {1: [], 2: [], 3: []} 
metas_virtuales = {1: None, 2: None, 3: None} 
lado_global = 1 
sockets_activos = {}

decisiones_maestro_cajas = {} 
decisiones_maestro_robots = {}
DISTANCIA_COMPROMISO = 0.35 

ultima_ruta_calculada = {1: "", 2: "", 3: ""}
lock_transmision = threading.Lock()

csv_archivos = {}
csv_escritores = {}
tiempo_inicio_experimento = 0

historial_posiciones = {1: [], 2: [], 3: []}
cooldown_escape = {1: 0, 2: 0, 3: 0}
mapa_global_obstaculos = [] 

# [x y theta vx vy vtheta] 
class FiltroKalmanBot:
    def __init__(self):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array([[1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([
            [1,0,0,1,0,0], [0,1,0,0,1,0], [0,0,1,0,0,1],
            [0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,0,0,0,1]], np.float32)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.05

    def predecir_corregir(self, x, y, theta):
        medicion = np.array([[np.float32(x)], [np.float32(y)], [np.float32(theta)]])
        self.kf.correct(medicion)
        pred = self.kf.predict()
        return pred[0][0], pred[1][0], pred[2][0]

filtros_bots = {1: FiltroKalmanBot(), 2: FiltroKalmanBot(), 3: FiltroKalmanBot()}

# Metas dinamicas
def poner_metas(event, x, y, flags, param):
    global metas_virtuales, lado_global
    if event == cv2.EVENT_LBUTTONDBLCLK:
        x_n, y_n = x / lado_global, 1.0 - (y / lado_global)
        if 0.0 <= x_n <= 1.0 and 0.0 <= y_n <= 1.0: metas_virtuales[1] = (x_n, y_n)
    elif event == cv2.EVENT_RBUTTONDBLCLK:
        x_n, y_n = x / lado_global, 1.0 - (y / lado_global)
        if 0.0 <= x_n <= 1.0 and 0.0 <= y_n <= 1.0: metas_virtuales[2] = (x_n, y_n)
    elif event == cv2.EVENT_MBUTTONDBLCLK: 
        x_n, y_n = x / lado_global, 1.0 - (y / lado_global)
        if 0.0 <= x_n <= 1.0 and 0.0 <= y_n <= 1.0: metas_virtuales[3] = (x_n, y_n)

# Cuadrantes
def actualizar_pendientes(inicio, meta, archivo_ruta):
    meta_x_segura, meta_y_segura = meta[0], meta[1]
    if abs(meta_x_segura - inicio[0]) < 0.015: meta_x_segura += 0.015
    if abs(meta_y_segura - inicio[1]) < 0.015: meta_y_segura += 0.015

    dx, dy = meta_x_segura - inicio[0], meta_y_segura - inicio[1]

    if (dx >= 0 and dy >= 0) or (dx < 0 and dy < 0):
        m1_dinamico, m2_dinamico = M1_CUADRANTES_1_3, M2_CUADRANTES_1_3
    else:
        m1_dinamico, m2_dinamico = M1_CUADRANTES_2_4, M2_CUADRANTES_2_4

    try:
        if os.path.exists(archivo_ruta):
            with open(archivo_ruta, 'r') as f: lineas = f.read().splitlines()
        else: lineas = []

        if len(lineas) < 8:
            lineas = [RADIO_S, m1_dinamico, m2_dinamico, PASOS_MAXIMOS, "0.0", "0.0", "1.0", "1.0"]

        lineas[1], lineas[2] = m1_dinamico, m2_dinamico
        lineas[4], lineas[5] = f"{inicio[0]:.6f}", f"{inicio[1]:.6f}"
        lineas[6], lineas[7] = f"{meta_x_segura:.6f}", f"{meta_y_segura:.6f}"
        
        with open(archivo_ruta, 'w') as f: f.write("\n".join(lineas) + "\n")
    except Exception: pass

def procesar_y_transmitir_trayectoria(bot_id, config, heading, lado_cuadrado, archivo_tray):
    global ultima_ruta_calculada, lock_transmision
    if not os.path.exists(archivo_tray): 
        rutas_memoria[bot_id] = []
        return
        
    try:
        with open(archivo_tray, 'r') as f:
            datos_texto = f.read()
            puntos = []
            for linea in datos_texto.strip().split('\n'):
                partes = linea.strip().split()
                if len(partes) >= 2:
                    puntos.append([int(float(partes[0]) * lado_cuadrado), 
                                   int((1.0 - float(partes[1])) * lado_cuadrado)])
            if len(puntos) > 1: rutas_memoria[bot_id] = np.array(puntos, np.int32).reshape((-1, 1, 2))
            else: rutas_memoria[bot_id] = []
        
        if datos_texto:
            with lock_transmision:
                ultima_ruta_calculada[bot_id] = datos_texto
    except Exception: pass

def planificacion_asincrona(bot_id, config, inicio, meta, obs_para_este_bot, heading, lado_cuadrado):
    global hilos_ocupados
    
    if hilos_ocupados[bot_id]: return 
    hilos_ocupados[bot_id] = True

    def tarea():
        try:
            archivo_pend = os.path.join(RUTA_BASE, f"pendientes_{bot_id}.txt")
            archivo_obs = os.path.join(RUTA_BASE, f"obstaculos_{bot_id}.txt")
            archivo_tray = os.path.join(RUTA_BASE, f"tray_{bot_id}.txt")

            actualizar_pendientes(inicio, meta, archivo_pend)
            
            try:
                with open(archivo_obs, 'w') as f:
                    for obs in obs_para_este_bot:
                        k_original = obs[3] 
                        
                        if abs(k_original) >= abs(MAGNITUD_K_ESTATICO) * 0.9 and abs(k_original) <= abs(MAGNITUD_K_ESTATICO) * 1.1:
                            if bot_id == 1:
                                k_final = abs(k_original)  # Robot 1 
                            else:
                                k_final = -abs(k_original) # Robot 2 
                        else:
                            k_final = k_original
                            
                        f.write(f"{obs[0]:.6f}\t{obs[1]:.6f}\t{obs[2]:.6f}\t{k_final:.5f}\n")
            except: pass
            
            if os.path.exists(archivo_tray): os.remove(archivo_tray)
            
            if os.path.exists(EJECUTABLE_HPPM):
                try: subprocess.run([EJECUTABLE_HPPM, str(bot_id)], cwd=RUTA_BASE, creationflags=subprocess.CREATE_NO_WINDOW, timeout=2)
                except: pass
            
            procesar_y_transmitir_trayectoria(bot_id, config, heading, lado_cuadrado, archivo_tray)
        finally:
            hilos_ocupados[bot_id] = False

    threading.Thread(target=tarea, daemon=True).start()

def detectar_en_vivo():
    global MODO_AUTOMATICO, lado_global, sockets_activos, mapa_global_obstaculos, ultima_ruta_calculada
    global csv_archivos, csv_escritores, tiempo_inicio_experimento
    global decisiones_maestro_cajas, decisiones_maestro_robots

    cap = cv2.VideoCapture(1)
    
    cv2.namedWindow("Visual Servoing Enjambre")
    cv2.setMouseCallback("Visual Servoing Enjambre", poner_metas)

    print("\n=== SISTEMA MULTI-AGENTE (NÚCLEO FINAL) ===")
    print(" - DOBLE CLIC IZQUIERDO: Fijar meta Robot 1")
    print(" - DOBLE CLIC DERECHO: Fijar meta Robot 2")
    print(" - DOBLE CLIC CENTRAL (Rueda): Fijar meta Robot 3")
    print(" - PRESIONA 'A': Iniciar transmisión continua y GRABAR DATOS (CSV)")
    print(" - PRESIONA 'Q': Salir y cerrar puertos\n")

    frame_count = 0
    modo_previo = False
    parametros_aruco = cv2.aruco.DetectorParameters()

    usar_calibracion = False
    mapx, mapy = None, None
    if os.path.exists(ARCHIVO_CALIBRACION):
        try:
            with np.load(ARCHIVO_CALIBRACION) as data:
                mtx, dist = data['mtx'], data['dist']
            ret_temp, frame_temp = cap.read()
            if ret_temp:
                h_cam, w_cam = frame_temp.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w_cam, h_cam), 1, (w_cam, h_cam))
                mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w_cam, h_cam), 5)
                usar_calibracion = True
        except: pass

    while True:
        ret, frame_crudo = cap.read()
        if not ret: break
        frame_count += 1
        
        if usar_calibracion: frame = cv2.remap(frame_crudo, mapx, mapy, cv2.INTER_LINEAR)
        else: frame = frame_crudo.copy()

        lado_global = min(frame.shape[0], frame.shape[1])
        img_cuadrada = frame[(frame.shape[0]-lado_global)//2:(frame.shape[0]+lado_global)//2, 
                             (frame.shape[1]-lado_global)//2:(frame.shape[1]+lado_global)//2].copy()

        gris = cv2.medianBlur(cv2.cvtColor(img_cuadrada, cv2.COLOR_BGR2GRAY), 7)

        posiciones_bots = {}   
        posiciones_metas_fisicas = {}  

        try: det = cv2.aruco.ArucoDetector(diccionario_aruco, parametros_aruco)
        except AttributeError: det = None
        
        if det: esquinas, ids, _ = det.detectMarkers(img_cuadrada)
        else: esquinas, ids, _ = cv2.aruco.detectMarkers(img_cuadrada, diccionario_aruco, parameters=parametros_aruco)

        if ids is not None:
            for i in range(len(ids)):
                id_actual, esq = ids[i][0], esquinas[i][0]
                c_x, c_y = int(np.mean(esq[:, 0])), int(np.mean(esq[:, 1]))
                x_n, y_n = c_x / lado_global, 1.0 - (c_y / lado_global)

                if id_actual in CONFIG_ROBOTS:
                    f_x, f_y = (esq[0][0] + esq[1][0]) / 2.0, (esq[0][1] + esq[1][1]) / 2.0
                    heading_crudo = math.atan2(c_y - f_y, f_x - c_x)
                    
                    x_k, y_k, heading_k = filtros_bots[id_actual].predecir_corregir(x_n, y_n, heading_crudo)
                    posiciones_bots[id_actual] = (x_k, y_k, heading_k)
                    
                    if MODO_AUTOMATICO:
                        historial_posiciones[id_actual].append((x_k, y_k))
                        if len(historial_posiciones[id_actual]) > 60: 
                            historial_posiciones[id_actual].pop(0)
                    else:
                        historial_posiciones[id_actual].clear()
                    
                    cv2.polylines(img_cuadrada, [esq.astype(int)], True, CONFIG_ROBOTS[id_actual]["color"], 2)
                    pt_flecha = (int(c_x + 50 * math.cos(heading_k)), int(c_y - 50 * math.sin(heading_k)))
                    cv2.arrowedLine(img_cuadrada, (c_x, c_y), pt_flecha, (0, 255, 0), 3, tipLength=0.3)
                    cv2.putText(img_cuadrada, f"B{id_actual}", (c_x-10, c_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, CONFIG_ROBOTS[id_actual]["color"], 2)
                
                elif id_actual in [config["id_meta"] for config in CONFIG_ROBOTS.values()]:
                    posiciones_metas_fisicas[id_actual] = (x_n, y_n)
                    cv2.polylines(img_cuadrada, [esq.astype(int)], True, (0, 255, 0), 3)
                    cv2.putText(img_cuadrada, f"M{id_actual}", (c_x-15, c_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        circulos = cv2.HoughCircles(gris, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=100, param2=30, minRadius=15, maxRadius=100)

        for obs in mapa_global_obstaculos:
            obs[3] -= 1 

        if circulos is not None:
            for (x, y, r) in np.round(circulos[0, :]).astype("int"):
                x_n, y_n, r_n = x / lado_global, 1.0 - (y / lado_global), r / lado_global
                
                es_chasis = False
                for datos in posiciones_bots.values():
                    if math.hypot(datos[0] - x_n, datos[1] - y_n) < (RADIO_ROBOT * 2.5): 
                        es_chasis = True
                        break
                if es_chasis: continue 

                es_meta = False
                for meta_pos in posiciones_metas_fisicas.values():
                    if math.hypot(meta_pos[0] - x_n, meta_pos[1] - y_n) < (RADIO_ROBOT * 2.0):
                        es_meta = True
                        break
                # También podemos revisar las metas virtuales (clics)
                for meta_virt in metas_virtuales.values():
                    if meta_virt is not None:
                        if math.hypot(meta_virt[0] - x_n, meta_virt[1] - y_n) < (RADIO_ROBOT * 2.0):
                            es_meta = True
                            break
                if es_meta: continue
                
                encontrado = False
                for obs in mapa_global_obstaculos:
                    if math.hypot(obs[0] - x_n, obs[1] - y_n) < 0.1: 
                        obs[0] = obs[0] * 0.9 + x_n * 0.1
                        obs[1] = obs[1] * 0.9 + y_n * 0.1
                        obs[2] = r_n
                        obs[3] = min(obs[3] + 10, 150) 
                        encontrado = True
                        break
                
                if not encontrado:
                    mapa_global_obstaculos.append([x_n, y_n, r_n, 20])

        mapa_global_obstaculos = [obs for obs in mapa_global_obstaculos if obs[3] > 0]

        obstaculos_estaticos_agrupados = []
        for obs in mapa_global_obstaculos:
            if obs[3] > 15: 
                obstaculos_estaticos_agrupados.append([obs[0], obs[1], obs[2] + RADIO_INFLADO, abs(MAGNITUD_K_ESTATICO)])
                
                px, py = int(obs[0] * lado_global), int((1.0 - obs[1]) * lado_global)
                r_px = int((obs[2] + RADIO_INFLADO) * lado_global)
                cv2.circle(img_cuadrada, (px, py), r_px, (0, 255, 255), 2)
                cv2.circle(img_cuadrada, (px, py), 2, (0, 0, 255), -1)

        if frame_count % FRECUENCIA_RUTAS == 0: 
            for bot_id, config in CONFIG_ROBOTS.items():
                if cooldown_escape[bot_id] > 0:
                    cooldown_escape[bot_id] -= FRECUENCIA_RUTAS
                    if cooldown_escape[bot_id] <= 0:
                        historial_posiciones[bot_id].clear() 
                    continue 

                meta_final = metas_virtuales[bot_id] 
                if meta_final is None and config["id_meta"] in posiciones_metas_fisicas:
                    meta_final = posiciones_metas_fisicas[config["id_meta"]]

                if bot_id in posiciones_bots and meta_final is not None:
                    x_r, y_r, heading = posiciones_bots[bot_id]

                    distancia_actual_meta = math.hypot(meta_final[0] - x_r, meta_final[1] - y_r)
                    if distancia_actual_meta <= DISTANCIA_LLEGADA:
                        with lock_transmision:
                            ultima_ruta_calculada[bot_id] = f"{x_r:.6f} {y_r:.6f}\n{x_r:.6f} {y_r:.6f}"
                        rutas_memoria[bot_id] = [] 
                        continue

                    dx_tray = meta_final[0] - x_r
                    dy_tray = meta_final[1] - y_r
                    obs_para_este_bot = []

                    invertir_k = not ((dx_tray >= 0 and dy_tray >= 0) or (dx_tray < 0 and dy_tray < 0))

                    ceder_paso = False
                    compañeros_activos = [] 
                    
                    for otro_id, datos_otro in posiciones_bots.items():
                        if otro_id < bot_id: 
                            x_otro, y_otro, _ = datos_otro
                            vec_x, vec_y = x_otro - x_r, y_otro - y_r
                            dist_entre_bots = math.hypot(vec_x, vec_y)

                            if dist_entre_bots < RADIO_ROBOT * 2.2:
                                ceder_paso = True
                                break

                            radio_seguro = min(RADIO_ROBOT + RADIO_INFLADO_ROBOT, dist_entre_bots * 0.85)
                            
                            Z_ideal_otro = dx_tray * (y_otro - y_r) - dy_tray * (x_otro - x_r)
                            
                            if abs(Z_ideal_otro) < 0.01:
                                Z_ideal_otro = 0.01 if bot_id == 1 else -0.01

                            k_ideal_robot = (-abs(K_ROBOT) if Z_ideal_otro > 0 else abs(K_ROBOT)) if not invertir_k else (abs(K_ROBOT) if Z_ideal_otro > 0 else -abs(K_ROBOT))
                            
                            clave_robot = f"{otro_id}_{bot_id}"
                            if bot_id == 1:
                                decisiones_maestro_robots[clave_robot] = k_ideal_robot
                                k_robot_dinamico = k_ideal_robot
                            else:
                                if clave_robot in decisiones_maestro_robots:
                                    k_robot_dinamico = -decisiones_maestro_robots[clave_robot]
                                else:
                                    k_robot_dinamico = k_ideal_robot
                            
                            compañeros_activos.append({'x': x_otro, 'y': y_otro, 'radio': radio_seguro, 'Z': Z_ideal_otro})
                            obs_para_este_bot.append([x_otro, y_otro, radio_seguro, k_robot_dinamico])
                    
                    if ceder_paso:
                        with lock_transmision:
                            ultima_ruta_calculada[bot_id] = f"{x_r:.6f} {y_r:.6f}\n{x_r:.6f} {y_r:.6f}"
                        continue 

                    obs_cercano = None
                    d_min = float('inf')

                    for idx, obs_est in enumerate(obstaculos_estaticos_agrupados):
                        cx, cy, r_inf, k_mag = obs_est
                        d = math.hypot(cx - x_r, cy - y_r)
                        if d < d_min: d_min = d; obs_cercano = obs_est

                        Z_estatico_ideal = dx_tray * (cy - y_r) - dy_tray * (cx - x_r)
                        for comp in compañeros_activos:
                            if math.hypot(cx - comp['x'], cy - comp['y']) < (r_inf + comp['radio'] + RADIO_INFLADO):
                                Z_estatico_ideal = comp['Z']; break
                        
                        if abs(Z_estatico_ideal) < 0.01: 
                            Z_estatico_ideal = 0.01 if bot_id == 1 else -0.01

                        k_ideal_estatico = (-abs(k_mag) if Z_estatico_ideal > 0 else abs(k_mag)) if not invertir_k else (abs(k_mag) if Z_estatico_ideal > 0 else -abs(k_mag))
                        
                        if bot_id == 1:
                            decisiones_maestro_cajas[idx] = k_ideal_estatico
                            k_dinamico = k_ideal_estatico
                        else:
                            if idx in decisiones_maestro_cajas:
                                k_dinamico = -decisiones_maestro_cajas[idx]
                            else:
                                k_dinamico = k_ideal_estatico
                        
                        obs_para_este_bot.append([cx, cy, r_inf, k_dinamico])

                    if MODO_AUTOMATICO and len(historial_posiciones[bot_id]) == 60 and not ceder_paso:
                        pos_vieja = historial_posiciones[bot_id][0]
                        if math.hypot(x_r - pos_vieja[0], y_r - pos_vieja[1]) < 0.04 and math.hypot(meta_final[0] - x_r, meta_final[1] - y_r) > 0.15 and d_min < 0.15:
                            vx, vy = meta_final[0] - x_r, meta_final[1] - y_r
                            norma = max(math.hypot(vx, vy), 0.001)
                            x_esc, y_esc = max(0.05, min(0.95, x_r + (vx / norma) * 0.15)), max(0.05, min(0.95, y_r + (vy / norma) * 0.15))
                            
                            puntos_escape = [[int(x_r * lado_global), int((1.0 - y_r) * lado_global)], [int(x_esc * lado_global), int((1.0 - y_esc) * lado_global)]]
                            rutas_memoria[bot_id] = np.array(puntos_escape, np.int32).reshape((-1, 1, 2))
                            
                            with lock_transmision:
                                ultima_ruta_calculada[bot_id] = f"{x_r:.6f} {y_r:.6f}\n{x_esc:.6f} {y_esc:.6f}"
                            cooldown_escape[bot_id] = 60
                            continue 

                    planificacion_asincrona(bot_id, config, (x_r, y_r), meta_final, obs_para_este_bot, heading, lado_global)

        if MODO_AUTOMATICO:
            for bot_id, config in CONFIG_ROBOTS.items():
                if bot_id in posiciones_bots:
                    x_r, y_r, heading_k = posiciones_bots[bot_id]
                    
                    if bot_id in csv_escritores:
                        t_actual = time.time() - tiempo_inicio_experimento
                        csv_escritores[bot_id].writerow([round(t_actual, 4), round(x_r, 6), round(y_r, 6), round(heading_k, 6)])

                    with lock_transmision:
                        texto_ruta_actual = ultima_ruta_calculada.get(bot_id, "")
                    
                    if texto_ruta_actual:
                        paquete = f"HEADING {heading_k:.6f}\n{texto_ruta_actual}\nEND\n"
                        
                        if bot_id not in sockets_activos:
                            try:
                                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                                s.settimeout(0.5) 
                                s.connect((config["ip"], config["puerto"]))
                                s.settimeout(0.05) 
                                sockets_activos[bot_id] = s
                            except Exception:
                                pass
                        
                        if bot_id in sockets_activos:
                            try: 
                                sockets_activos[bot_id].sendall(paquete.encode('utf-8'))
                            except:
                                sockets_activos[bot_id].close()
                                del sockets_activos[bot_id]
                                
        elif modo_previo:
            for bot_id in CONFIG_ROBOTS:
                if bot_id in posiciones_bots and bot_id in sockets_activos:
                    x_r, y_r, heading_k = posiciones_bots[bot_id]
                    paquete_paro = f"HEADING {heading_k:.6f}\n{x_r:.6f} {y_r:.6f}\n{x_r:.6f} {y_r:.6f}\nEND\n"
                    try: sockets_activos[bot_id].sendall(paquete_paro.encode('utf-8'))
                    except: pass
        
        modo_previo = MODO_AUTOMATICO

        for bot_id, puntos in rutas_memoria.items():
            if len(puntos) > 0 and bot_id in CONFIG_ROBOTS:
                cv2.polylines(img_cuadrada, [puntos], False, CONFIG_ROBOTS[bot_id]["color"], 4, cv2.LINE_AA)

        for bot_id, pos_virtual in metas_virtuales.items():
            if pos_virtual:
                px, py = int(pos_virtual[0] * lado_global), int((1.0 - pos_virtual[1]) * lado_global)
                cv2.drawMarker(img_cuadrada, (px, py), CONFIG_ROBOTS[bot_id]["color"], cv2.MARKER_CROSS, 20, 3)

        texto_estado = "MULTI-AGENTE ACTIVO [GRABANDO CSV]" if MODO_AUTOMATICO else "PAUSADO (Presiona 'A')"
        cv2.putText(img_cuadrada, texto_estado, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if MODO_AUTOMATICO else (0, 0, 255), 2)
        cv2.imshow("Visual Servoing Enjambre", img_cuadrada)

        tecla = cv2.waitKey(1) & 0xFF
        
        if tecla == ord('a') or tecla == ord('A'): 
            MODO_AUTOMATICO = not MODO_AUTOMATICO
            if MODO_AUTOMATICO:
                tiempo_inicio_experimento = time.time()
                decisiones_maestro_cajas.clear()
                decisiones_maestro_robots.clear()
                
                for bot_id in CONFIG_ROBOTS:
                    nombre_archivo = os.path.join(RUTA_BASE, f"registro_real_bot{bot_id}.csv")
                    f = open(nombre_archivo, 'w', newline='')
                    csv_archivos[bot_id] = f
                    writer = csv.writer(f)
                    writer.writerow(["Tiempo(s)", "X_Real", "Y_Real", "Heading_Rad"])
                    csv_escritores[bot_id] = writer
                print("\n[REC] Grabación de datos CSV INICIADA.")
            else:
                for bot_id in list(csv_archivos.keys()):
                    csv_archivos[bot_id].close()
                csv_archivos.clear()
                csv_escritores.clear()
                print("\n[STOP] Grabación de datos CSV DETENIDA y guardada.")
                
        elif tecla == ord('q') or tecla == ord('Q'): 
            break

    for s in sockets_activos.values(): s.close()
    for f in csv_archivos.values(): f.close() 
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detectar_en_vivo()