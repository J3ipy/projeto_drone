import time
import threading
import pickle
from collections import deque
from datetime import datetime
import numpy as np
from djitellopy import Tello
import cv2

# Letras válidas dos QR codes
VALID_QR = {'E', 'D', 'C', 'B', 'A'}

# Buffer para armazenar sempre o frame mais recente (já redimensionado)
frame_buffer = deque(maxlen=1)

def frame_reader(tello):
    """Thread dedicada só para leitura e downscale do stream."""
    tello.streamon()
    while True:
        frame = tello.get_frame_read().frame
        if frame is not None:
            # Reduz para 320×240 px para otimização
            small = cv2.resize(frame, (320, 240))
            frame_buffer.append(small)

# Conectar e configurar Tello
tello = Tello()
tello.connect()
print(f"[INFO] Bateria inicial: {tello.get_battery()}%")
tello.takeoff()
time.sleep(2)

# Inicia thread de leitura
reader_thread = threading.Thread(target=frame_reader, args=(tello,), daemon=True)
reader_thread.start()

# Detector de QR mais rápido do OpenCV
qr_detector = cv2.QRCodeDetector()

infos = []
start_time = time.time()
show_counter = 0
detect_counter = 0

try:
    while True:
        # Pega o frame mais recente do buffer
        if not frame_buffer:
            continue
        frame = frame_buffer[-1]
        
        # Mostrar stream apenas a cada 2 frames
        show_counter += 1
        if show_counter % 2 == 0:
            cv2.imshow('Tello Stream', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
        
        # Fazer detecção de QR apenas a cada 3 frames
        detect_counter += 1
        if detect_counter % 3 == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            # Região de interesse central de 200×200 px
            cx, cy = w // 2, h // 2
            roi = gray[cy-100:cy+100, cx-100:cx+100]
            
            data, pts, _ = qr_detector.detectAndDecode(roi)
            if data and data in VALID_QR and pts is not None:
                print(f"[INFO] QR detectado: {data}")
                
                # pts vem com shape (4,1,2): converte para (4,2)
                pts = pts.squeeze().reshape(-1, 2)
                # Ajusta coords da ROI para coords no frame inteiro
                offset_x, offset_y = cx - 100, cy - 100
                pts += np.array([offset_x, offset_y])
                
                x1, y1 = pts[0]
                x3, y3 = pts[2]
                distancia = np.hypot(x3 - x1, y3 - y1)
                centrox = (x3 + x1) / 2

                # Ajuste para centralizar e se aproximar
                if distancia < 150:
                    tello.move_forward(20)
                    print("[INFO] Aproximando do QR...")
                    continue
                if distancia > 200:
                    tello.move_back(20)
                    print("[INFO] Afastando do QR...")
                    continue
                if centrox < (cx - 20):
                    tello.move_left(20)
                    print("[INFO] Ajustando para a esquerda...")
                    continue
                if centrox > (cx + 20):
                    tello.move_right(20)
                    print("[INFO] Ajustando para a direita...")
                    continue
                
                # Quando estiver centralizado, avança para procurar o próximo
                print("[INFO] QR centralizado, movendo para o próximo...")
                tello.move_forward(50)
                time.sleep(2)

                # Log da detecção
                altura_m = tello.get_height() / 100.0
                elapsed = time.time() - start_time
                infos.append((data, altura_m, elapsed))

except KeyboardInterrupt:
    print("[INFO] Interrompido pelo usuário.")

# Pousar e encerrar
print("[INFO] Pousando...")
tello.land()
bat_final = tello.get_battery()
print(f"[INFO] Bateria final: {bat_final}% (inicial {tello.get_battery()}%)")

# Pergunta para salvar dados
if input('Salvar os dados desse teste? (s/n): ').lower() == 's':
    now = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'dados_de_voo_{now}.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(infos, f)
    print(f"[INFO] Dados salvos em: {filename}")
else:
    print("[INFO] Teste descartado.")

tello.streamoff()
cv2.destroyAllWindows()
tello.end()
