import cv2
import torch
import time
import snap7
import struct

camera_index = 0

def load_yolov5_model(weights_path, device='cpu'):
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path, force_reload=True)
    model.to(device).eval()
    return model

def capture_roi(frame, xyxy):
    x1, y1, x2, y2 = xyxy
    roi = frame[y1:y2, x1:x2]
    return roi

def get_detection_center_mass(detection):
    center_x = (detection[0] + detection[2]) / 2
    center_y = (detection[1] + detection[3]) / 2
    return center_x, center_y

def validacion(actual, anterior):
    if (actual != 0 and anterior != 9) and (anterior == 9 and actual == 10):
        if (actual < anterior or actual == 10):
            actual = anterior
    if actual == 10 or actual == "10":
        actual = anterior
    return str(actual)

# Configura la conexión con el PLC
ip = '192.168.1.12'
rack = 0
slot = 1

# Instancia de cliente
cliente = snap7.client.Client()

# Conexión a PLC
cliente.connect(ip, rack, slot)

if cliente.get_connected():
    print("Conexión exitosa al PLC")
else:
    print("Error al conectar al PLC")

numeroFinal = ""
if __name__ == "__main__":
    weights_path = r"best.pt"  # Especifica la ruta a los pesos del modelo preentrenado
    model = load_yolov5_model(weights_path)

    cap = cv2.VideoCapture(camera_index)  # Abre la cámara
    roi = None
    show_roi = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)  # Realiza la detección en el frame actual
        detections = results.pred[0]  # Obtiene las detecciones

        for *xyxy, conf, cls in detections:
            label = f'{model.names[int(cls)]} {conf:.2f}'
            xyxy = [int(i) for i in xyxy]
            frame = cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (255, 0, 0), 2)
            frame = cv2.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            key = cv2.waitKey(4)
            if key:
                roi = capture_roi(frame, xyxy)
                show_roi = True
                break

        cv2.imshow('YOLOv5 Detection', frame)

        if show_roi and roi is not None:
            break

        if cv2.waitKey(0) == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit()

    roi_coord = [xyxy[0], xyxy[1], xyxy[2], xyxy[3]]
    cap.release()
    cv2.destroyAllWindows()

    cap = cv2.VideoCapture(camera_index)
    repetDetection = 0
    roi2 = None

    if label.startswith("analogico"):
        cv2.namedWindow('NUMEROS', cv2.WINDOW_NORMAL)

        print("Medidor analogico")
        weights_path_numbers = r"digital.pt"  # AQUÍ SE CARGA EL MODELO ENTRENADO PARA DETECCIÓN DE NÚMEROS ANALÓGICOS
        model = load_yolov5_model(weights_path_numbers)

        repet = 0
        for i in range(4):
            print("Validacion " + str(i + 1))
            ret, frame = cap.read()
            frame = frame[int(roi_coord[1]):int(roi_coord[3]), int(roi_coord[0]):int(roi_coord[2])]
            if not ret:
                break
            frame = frame.copy()
            results = model(frame)
            detections = results.pred[0]
            classes = detections[:, -1].int()
            sorted_indices = detections[:, 0].argsort()
            classes_sorted = classes[sorted_indices]
            detected_classes_list = classes_sorted.tolist()
            numeroDetecciones = int(len(detected_classes_list))
            repetDetection += numeroDetecciones
            repet += 1

        cantNumeros = int(repetDetection / repet)
        detectedClassAnt = [0] * cantNumeros

        print("La cantidad de numeros es: " + str(cantNumeros))
        time.sleep(3)

        numero_final_anterior = 0

        while True:
            ret, frame = cap.read()
            frame = frame[int(roi_coord[1]):int(roi_coord[3]), int(roi_coord[0]):int(roi_coord[2])]
            if not ret:
                break
            frame = frame.copy()
            results = model(frame)
            detections = results.pred[0]

            for *xyxy, conf, cls in detections:
                label = f'{model.names[int(cls)]} {conf:.2f}'
                xyxy = [int(i) for i in xyxy]
                frame = cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (255, 0, 0), 2)
                frame = cv2.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 0, 0), 2)

            classes = detections[:, -1].int()
            sorted_indices = detections[:, 0].argsort()
            classes_sorted = classes[sorted_indices]
            detected_classes_list = classes_sorted.tolist()
            numeroDetecciones = int(len(detected_classes_list))

            if (numeroDetecciones > cantNumeros) or (numeroDetecciones < cantNumeros):
                continue

            numeroFinal = ""
            for secuencia in range(cantNumeros):
                if (detected_classes_list[secuencia] == 10):
                    detected_classes_list[secuencia] = 0
                numeroFinal += validacion(int(detected_classes_list[secuencia]),
                                          int(detectedClassAnt[secuencia]))

                detectedClassAnt[secuencia] = detected_classes_list[secuencia]

            numeroFinal = int(numeroFinal)
            valorFinal = int(numeroFinal)

            if numeroFinal < numero_final_anterior:
                numeroFinal = numero_final_anterior

            numero_final_anterior = numeroFinal

            try:
                db_number = 1
                start = 4
                valor = valorFinal
                data_word = bytearray(struct.pack('>I', valor))
                cliente.db_write(db_number, start, data_word)
                print(f"Escritura en DB {db_number}")
                print('escribiendo')
                print(valorFinal)
            except Exception as e:
                print("Error al conectar al PLC")
            finally:
                pass

            cv2.imshow('NUMEROS', frame)

            if cv2.waitKey(1) == ord('q'):
                numeroFinal = 0
                numero_final_anterior = 0
                cap.release()
                cv2.destroyAllWindows()
                exit()

    if label.startswith("digital"):
        print("Medidor digital")
        cv2.namedWindow('NUMEROS', cv2.WINDOW_NORMAL)
        weights_path_numbers = r"digital.pt"  # AQUÍ SE CARGA EL MODELO ENTRENADO PARA DETECCIÓN DE NÚMEROS DIGITALES
        model = load_yolov5_model(weights_path_numbers)

        repet = 0
        for i in range(4):
            print("Validacion " + str(i + 1))
            ret, frame = cap.read()
            frame = frame[int(roi_coord[1]):int(roi_coord[3]), int(roi_coord[0]):int(roi_coord[2])]
            if not ret:
                break
            frame = frame.copy()
            results = model(frame)
            detections = results.pred[0]
            classes = detections[:, -1].int()
            sorted_indices = detections[:, 0].argsort()
            classes_sorted = classes[sorted_indices]
            detected_classes_list = classes_sorted.tolist()
            numeroDetecciones = int(len(detected_classes_list))
            repetDetection += numeroDetecciones
            repet += 1

        cantNumeros = int(repetDetection / repet)
        detectedClassAnt = [0] * cantNumeros

        print("La cantidad de numeros digitales es: " + str(cantNumeros))
        time.sleep(2)

        numero_final_anterior = 0

        while True:
            ret, frame = cap.read()
            frame = frame[int(roi_coord[1]):int(roi_coord[3]), int(roi_coord[0]):int(roi_coord[2])]
            if not ret:
                break
            frame = frame.copy()
            results = model(frame)
            detections = results.pred[0]

            for *xyxy, conf, cls in detections:
                label = f'{model.names[int(cls)]} {conf:.2f}'
                xyxy = [int(i) for i in xyxy]
                frame = cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (255, 0, 0), 2)
                frame = cv2.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 0, 0), 2)

            classes = detections[:, -1].int()
            sorted_indices = detections[:, 0].argsort()
            classes_sorted = classes[sorted_indices]
            detected_classes_list = classes_sorted.tolist()
            numeroDetecciones = int(len(detected_classes_list))

            if (numeroDetecciones > cantNumeros) or (numeroDetecciones < cantNumeros):
                continue

            numeroFinal = ""
            for secuencia in range(cantNumeros):
                if (detected_classes_list[secuencia] == 10):
                    detected_classes_list[secuencia] = 0
                numeroFinal += validacion(int(detected_classes_list[secuencia]),
                                          int(detectedClassAnt[secuencia]))

                detectedClassAnt[secuencia] = detected_classes_list[secuencia]

            numeroFinal = int(numeroFinal)
            valorFinal = int(numeroFinal)

            if numeroFinal < numero_final_anterior:
                numeroFinal = numero_final_anterior

            numero_final_anterior = numeroFinal

            try:
                db_number = 1
                start = 4
                valor = valorFinal
                data_word = bytearray(struct.pack('>I', valor))
                cliente.db_write(db_number, start, data_word)
                print(f"Escritura en DB {db_number}")
                print('escribiendo')
                print(valorFinal)
            except Exception as e:
                print("Error al conectar al PLC")
            finally:
                pass

            cv2.imshow('NUMEROS', frame)

            if cv2.waitKey(1) == ord('q'):
                numeroFinal = 0
                numero_final_anterior = 0
                cap.release()
                cv2.destroyAllWindows()
                exit()

    cap.release()
    cv2.destroyAllWindows()
cap.release()
cv2.destroyAllWindows()
