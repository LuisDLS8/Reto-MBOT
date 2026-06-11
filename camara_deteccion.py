import cv2
import numpy as np
import serial
import time

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Serial
ser = serial.Serial('/dev/ttyUSB0',9600,timeout=1)
ser.reset_input_buffer()
time.sleep(2)

centro = 320
error_filtrado = 0
ultimo_comando = 0

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Recorte de imagen a analizar
    recorte = frame[150:480, :]

    # Realiza filtrado y reduce ruido
    gray = cv2.cvtColor(recorte,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(9, 9),0)
    _, thresh = cv2.threshold(gray,80,255,cv2.THRESH_BINARY_INV)
    kernel = np.ones((5, 5),np.uint8)
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel)

    # Canny
    canny = cv2.Canny(thresh,50,150)

    # Lineas de HOUGH
    lines = cv2.HoughLinesP(canny,1,np.pi / 180,threshold=40,minLineLength=60,maxLineGap=30)
    linea_detectada = False
    centros_x = []
    angulos = []

    # Filtrado de las lineas detectadas
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            longitud = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            if longitud > 80:
                x_centro = int((x1 + x2) / 2)
                centros_x.append(x_centro)
                angulo = np.degrees(np.arctan2(y2 - y1, x2 - x1))

                # Reorienta para que vertical sea 0° y no 90
                angulo = angulo + 90
                angulos.append(angulo)

    angulo_promedio = 0
    if len(angulos) > 0:
        angulo_promedio = np.mean(angulos)

    # Modo de control
    if (len(centros_x) > 0):
        linea_detectada = True
        cx = int(np.mean(centros_x))
        error_x = (cx - centro)

        # Filtro para evitar cambios bruscos
        error_filtrado = (0.85 * error_filtrado + 0.15 * error_x)
        comando = int(error_filtrado / 8)
        comando = max(-30,min(30,comando))

        if (abs(comando -ultimo_comando) > 1):
            ultimo_comando = comando

    #Envio a arduino
    if linea_detectada:
        print(ultimo_comando)
        ser.write((str(ultimo_comando) + "\n").encode())

    else:
        if (ultimo_comando > 3):
            print("DERECHA")
            ser.write(b"DER\n")

        elif (ultimo_comando < -3):
            print("IZQUIERDA")
            ser.write(b"IZQ\n")

        else:
            print("PARA")
            ser.write(b"PARA\n")

    # Mostrar camara
    cv2.imshow("Canny",canny)
    cv2.imshow("Camara",frame)
    tecla = (cv2.waitKey(1)& 0xFF)
    if tecla == 13:
        break

cap.release()
ser.close()
cv2.destroyAllWindows()