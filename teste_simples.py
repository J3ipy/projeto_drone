import time
import threading
import pickle
from collections import deque
from datetime import datetime
import cv2
import numpy as np
from djitellopy import Tello
import signal
import sys

# ---------- Parâmetros ----------
SEQUENCE = ['E', 'D', 'C', 'B', 'A']
MAX_HEIGHT_CM = 200
GAIN_X = 0.1      # ganho proporcional horizontal
GAIN_Y = 0.1      # ganho proporcional vertical
TH_X = 15         # limiar em pixels para centralização X
TH_Y = 15         # limiar em pixels para centralização Y
SIZE_MIN = 150    # tamanho mínimo em pixels do QR para captura
SIZE_MAX = 200    # tamanho máximo em pixels do QR para captura
MOVE_STEP = 20    # cm de movimento para correções discretas

# Buffer único de frame
frame_buffer = deque(maxlen=1)

# ----- Tratamento de Interrupção Segura -----
def handler(sig, frame):
    print("\n[INFO] Ctrl+C detectado! Pousando...")
    drone.land()
    drone.streamoff()
    cv2.destroyAllWindows()
    drone.end()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

def frame_reader(drone):
    drone.streamon()
    while True:
        frame = drone.get_frame_read().frame
        if frame is None:
            continue
        small = cv2.resize(frame, (320, 240))
        frame_buffer.append(small)

# Detecção simples de QR
def detect_qr(detector, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    cx, cy = w // 2, h // 2
    roi = gray[cy-100:cy+100, cx-100:cx+100]
    data, pts, _ = detector.detectAndDecode(roi)
    if data and pts is not None:
        pts = pts.squeeze().reshape(-1, 2) + np.array([cx-100, cy-100])
        x1, y1 = pts[0]
        x3, y3 = pts[2]
        center = ((x1 + x3)/2, (y1 + y3)/2)
        size = np.hypot(x3 - x1, y3 - y1)
        return data, center, size
    return None, None, None

# Script principal
if __name__ == '__main__':
    # Configuração inicial
    drone = Tello()
    drone.connect()
    print(f"[INFO] Bateria inicial: {drone.get_battery()}%")

    detector = cv2.QRCodeDetector()
    reader = threading.Thread(target=frame_reader, args=(drone,), daemon=True)
    reader.start()

    log = []  # lista de (QR, altura, timestamp)
    seq_idx = 0
    current_height = 0

    drone.takeoff()
    time.sleep(2)

    try:
        while seq_idx < len(SEQUENCE):
            if not frame_buffer:
                continue
            frame = frame_buffer[-1]
            h, w = frame.shape[:2]
            target = SEQUENCE[seq_idx]

            data, (cx, cy), size = detect_qr(detector, frame)

            if data == target:
                # cálculos de erro
                err_x = (w/2) - cx
                err_y = (h/2) - cy
                # ação proporcional
                if abs(err_x) > TH_X:
                    move = min(abs(int(GAIN_X * err_x)), MOVE_STEP)
                    if err_x > 0: drone.move_left(move)
                    else:        drone.move_right(move)
                    time.sleep(0.2)
                    continue
                if abs(err_y) > TH_Y:
                    move = min(abs(int(GAIN_Y * err_y)), MOVE_STEP)
                    if err_y > 0: drone.move_up(move)
                    else:         drone.move_down(move)
                    time.sleep(0.2)
                    continue
                # verificação de tamanho para captura
                if SIZE_MIN <= size <= SIZE_MAX:
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                    fname = f"QR_{data}_{ts}.jpg"
                    cv2.imwrite(fname, frame)
                    log.append((data, current_height, fname, time.time()))
                    print(f"[INFO] Capturado {data} em altura {current_height}cm: {fname}")
                    # subir para próxima camada
                    if current_height + MOVE_STEP <= MAX_HEIGHT_CM:
                        drone.move_up(MOVE_STEP)
                        current_height += MOVE_STEP
                        time.sleep(1)
                    seq_idx += 1
                    continue
            # se não detectou ou não era alvo, afasta um pouco
            drone.move_back(MOVE_STEP)
            time.sleep(1)

    except KeyboardInterrupt:
        print("[INFO] Interrompido pelo usuário")

    finally:
        print("[INFO] Finalizando... Pousando e encerrando recursos.")
        drone.land()
        print(f"[INFO] Bateria final: {drone.get_battery()}%")
        with open('log_simple.pkl', 'wb') as f:
            pickle.dump(log, f)
        drone.streamoff()
        cv2.destroyAllWindows()
        drone.end()
