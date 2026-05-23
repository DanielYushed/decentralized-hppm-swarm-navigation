import serial
import time
import math
import socket
import threading

PORT_SERIAL = '/dev/ttyUSB0' 
BAUD = 9600
ESCALA = 3500.0 

DISTANCIA_LLEGADA = 80.0    # mm 
KP = 1.5                # Constante Proporcional Lineal (Empuje hacia adelante)
KW = 70.0               # Constante Proporcional Angular (Fuerza de giro)
VEL_MAX = 200           # Límite superior seguro de la señal PWM
ANCHO_ROBOT = 145.0         # Distancia física entre las llantas

HOST = '0.0.0.0' 
PORT_TCP = 5005

nueva_trayectoria_texto = None
lock_trayectoria = threading.Lock()

def enviar_y_esperar(ser, comando_str):
    """Envía el comando y espera confirmación. 
       Con el nuevo comando 'V', la respuesta será casi instantánea."""
    ser.reset_input_buffer()
    ser.write(comando_str.encode('utf-8'))
    while True:
        if ser.in_waiting > 0 and 'K' in ser.read(ser.in_waiting).decode('utf-8', errors='ignore'):
            break
        time.sleep(0.002)

def servidor_wifi():
    """Hilo secundario inalterado. Recibe los datos de la cámara en el techo."""
    global nueva_trayectoria_texto
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        s.bind((HOST, PORT_TCP))
        s.listen()
        print(f"\n[OK] ROBOT LISTO Y ESCUCHANDO EN EL PUERTO {PORT_TCP}")
        
        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    buffer_red = ""
                    while True:
                        data = conn.recv(4096)
                        if not data: break
                        buffer_red += data.decode('utf-8')
                        
                        if "END" in buffer_red:
                            fragmentos = buffer_red.split("END")
                            ruta_mas_fresca = fragmentos[-2].strip()
                            
                            if ruta_mas_fresca:
                                with lock_trayectoria:
                                    nueva_trayectoria_texto = ruta_mas_fresca
                                    
                            buffer_red = fragmentos[-1]
            except Exception as e:
                print(f"Error en WiFi: {e}") 

try:
    print("Iniciando conexión Serial con el robot Scribbler S2...")
    ser = serial.Serial(PORT_SERIAL, BAUD, timeout=1)
    
    ser.dtr = False; ser.rts = False; time.sleep(0.1)
    ser.dtr = True;  ser.rts = True;  time.sleep(0.1)
    ser.dtr = False
    ser.reset_input_buffer()
    
    print("Esperando confirmación ('K') del hardware...")
    start_time = time.time()
    robot_listo = False
    while (time.time() - start_time) < 8:
        if ser.in_waiting > 0 and 'K' in ser.read(ser.in_waiting).decode('utf-8', errors='ignore'):
            robot_listo = True
            break

    if not robot_listo:
        print("Error: El robot no respondió.")
        exit()

    hilo_wifi = threading.Thread(target=servidor_wifi, daemon=True)
    hilo_wifi.start()

    while True:
        datos_a_procesar = None
        
        with lock_trayectoria:
            if nueva_trayectoria_texto is not None:
                datos_a_procesar = nueva_trayectoria_texto
                nueva_trayectoria_texto = None 
        
        if datos_a_procesar:
            lista_puntos = []
            angulo_brujula = 0.0 
            
            for linea in datos_a_procesar.split('\n'):
                linea = linea.strip()
                if not linea: continue
                
                if linea.startswith("HEADING"):
                    angulo_brujula = float(linea.split()[1])
                    continue
                    
                partes = linea.split()
                if len(partes) >= 2:
                    x_escalado = float(partes[0]) * ESCALA
                    y_escalado = float(partes[1]) * ESCALA
                    lista_puntos.append((x_escalado, y_escalado))

            if len(lista_puntos) < 2:
                continue

            x_actual = lista_puntos[0][0]
            y_actual = lista_puntos[0][1]
            
            x_obj = lista_puntos[1][0]
            y_obj = lista_puntos[1][1]

            dx = x_obj - x_actual
            dy = y_obj - y_actual
            error_dist = math.hypot(dx, dy)
            
            if error_dist < DISTANCIA_LLEGADA:
                enviar_y_esperar(ser, "V0,0,")
                continue

            angulo_obj = math.atan2(dy, dx)
            error_ang = angulo_obj - angulo_brujula
            error_ang = (error_ang + math.pi) % (2 * math.pi) - math.pi
            
            v = KP * error_dist
            w = KW * error_ang
            
            v = min(v, VEL_MAX)

            # Cinemática Inversa (De v,w a V_izq, V_der)
            vel_izq = int(v - (w * ANCHO_ROBOT / 2.0))
            vel_der = int(v + (w * ANCHO_ROBOT / 2.0))

            max_calculada = max(abs(vel_izq), abs(vel_der))
            if max_calculada > VEL_MAX:
                vel_izq = int((vel_izq / max_calculada) * VEL_MAX)
                vel_der = int((vel_der / max_calculada) * VEL_MAX)

            zona_muerta = 15
            if 0 < abs(vel_izq) < zona_muerta: vel_izq = zona_muerta * (1 if vel_izq > 0 else -1)
            if 0 < abs(vel_der) < zona_muerta: vel_der = zona_muerta * (1 if vel_der > 0 else -1)

            enviar_y_esperar(ser, f"V{vel_izq},{vel_der},")
            
            time.sleep(0.05) 
            
        else:
            time.sleep(0.01)

except serial.SerialException:
    print(f"Error crítico: No se encuentra el puerto {PORT_SERIAL}.")
except KeyboardInterrupt:
    print("\nDetenido manualmente.")
except Exception as e:
    print(f"Error general: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.reset_input_buffer()
        ser.write("V0,0,".encode('utf-8'))
        ser.close()
        print("Motores apagados y puerto cerrado de forma segura.")