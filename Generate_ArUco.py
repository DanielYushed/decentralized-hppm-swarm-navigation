import cv2
import os

ruta_base = "Enter the base path here"

try:
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    equipo = {
        1: "bot1.png",
        2: "bot2.png",
        3: "bot3.png"
    }
    
    for id_aruco, nombre_archivo in equipo.items():
        ruta_guardado = os.path.join(ruta_base, nombre_archivo)
        
        imagen_marcador = cv2.aruco.generateImageMarker(diccionario, id_aruco, 200)
        cv2.imwrite(ruta_guardado, imagen_marcador)
        
        print(f"ID Marker {id_aruco} saved as: {nombre_archivo}")

except AttributeError:
    print("Error: Check your installation of opencv-contrib-python")
