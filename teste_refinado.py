import time
import threading
import pickle
from collections import deque
from datetime import datetime
import numpy as np
import cv2
from enum import Enum, auto
from djitellopy import Tello
import signal
import sys

# --------- Estados da Máquina ---------
class State(Enum):
    SEARCH    = auto()     # Buscar QR específico
    ALIGN     = auto()     # Centralizar QR (X e Y)
    CAPTURE   = auto()     # Capturar foto
    MOVE_UP   = auto()     # Subir camada após captura
    MOVE_BACK = auto()     # Afastar se não achar QR
    FINISHED  = auto()     # Término

# --------- PID para alinhamento ---------
class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self._prev_err = 0.0
        self._integral = 0.0

    def update(self, measurement, dt):
        err = self.setpoint - measurement
        self._integral += err * dt
        deriv = (err - self._prev_err) / dt if dt > 0 else 0.0
        out = self.kp*err + self.ki*self._integral + self.kd*deriv
        self._prev_err = err
        return out

# --------- Filtro de Kalman 1D para posição ---------
class SimpleKalman:
    def __init__(self, q=1e-5, r=1e-2):
        self.q, self.r = q, r
        self.x = 0.0
        self.p = 1.0

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# --------- Leitura de frames em thread ---------
frame_buffer = deque(maxlen=1)

def frame_reader(tello):
    tello.streamon()
    while True:
        f = tello.get_frame_read().frame
        if f is None:
            continue
        small = cv2.resize(f, (320, 240))
        frame_buffer.append(small)

# --------- Controlador Principal ---------
class QRAutonomy:
    def __init__(self, sequence=['E','D','C','B','A'], max_height_cm=200):
        self.tello = Tello()
        self.tello.connect()

        # PID e Kalman para X e Y
        self.pid_x = PID(kp=0.4, ki=0.0, kd=0.2)
        self.pid_y = PID(kp=0.4, ki=0.0, kd=0.2)
        self.kalman_x = SimpleKalman()
        self.kalman_y = SimpleKalman()
        self.qr_detector = cv2.QRCodeDetector()

        # Sequência desejada de QRs
        self.sequence = sequence
        self.seq_idx = 0

        # Estado e altura
        self.state = State.SEARCH
        self.current_height = 0
        self.MAX_HEIGHT = max_height_cm

        # Timers e registro
        self.start_time = time.time()
        self._last_time = self.start_time
        self.infos = []

    def detect_qr(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        cx, cy = w//2, h//2
        roi = gray[cy-100:cy+100, cx-100:cx+100]
        data, pts, _ = self.qr_detector.detectAndDecode(roi)
        if data and pts is not None:
            pts = pts.squeeze().reshape(-1,2)
            pts += np.array([cx-100, cy-100])
            x1, y1 = pts[0]
            x3, y3 = pts[2]
            center_x = (x1 + x3) / 2
            center_y = (y1 + y3) / 2
            size = np.hypot(x3 - x1, y3 - y1)
            return data, center_x, center_y, size
        return None, None, None, None

    def take_photo(self, frame, data):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"QR_{data}_{ts}.jpg"
        cv2.imwrite(fname, frame)
        elapsed = time.time() - self.start_time
        self.infos.append((data, self.current_height, fname, elapsed))
        print(f"[INFO] Foto salva: {fname}")

    def run(self):
        reader = threading.Thread(target=frame_reader, args=(self.tello,), daemon=True)
        reader.start()

        print(f"[INFO] Bateria inicial: {self.tello.get_battery()}%")
        self.tello.takeoff()
        time.sleep(2)

        try:
            while True:
                if not frame_buffer:
                    continue
                frame = frame_buffer[-1]
                now = time.time()
                dt = now - self._last_time
                self._last_time = now

                target = self.sequence[self.seq_idx] if self.seq_idx < len(self.sequence) else None

                if self.state == State.SEARCH:
                    data, cx, cy, size = self.detect_qr(frame)
                    if data == target:
                        self.detected_cx = cx
                        self.detected_cy = cy
                        self.detected_size = size
                        self.state = State.ALIGN
                    else:
                        # se não achar target em 3s ajustados pela altura, avança
                        if now - self.start_time > 3 + self.current_height/20:
                            self.state = State.MOVE_BACK

                elif self.state == State.ALIGN:
                    err_x = (frame.shape[1] / 2) - self.detected_cx
                    err_y = (frame.shape[0] / 2) - self.detected_cy
                    fx = self.kalman_x.update(err_x)
                    fy = self.kalman_y.update(err_y)
                    ax = self.pid_x.update(fx, dt)
                    ay = self.pid_y.update(fy, dt)

                    if abs(fx) > 20:
                        step = min(abs(int(ax)), 20)
                        if ax > 0: self.tello.move_left(step)
                        else:      self.tello.move_right(step)

                    elif abs(fy) > 20:
                        step = min(abs(int(ay)), 20)
                        if ay > 0: self.tello.move_up(step)
                        else:      self.tello.move_down(step)

                    else:
                        self.state = State.CAPTURE

                elif self.state == State.CAPTURE:
                    self.take_photo(frame, target)
                    self.seq_idx += 1
                    if self.seq_idx >= len(self.sequence):
                        self.state = State.FINISHED
                    else:
                        self.state = State.MOVE_UP

                elif self.state == State.MOVE_UP:
                    if self.current_height + 20 <= self.MAX_HEIGHT:
                        self.tello.move_up(20)
                        self.current_height += 20
                        self.start_time = time.time()
                        self.state = State.SEARCH
                    else:
                        self.state = State.FINISHED

                elif self.state == State.MOVE_BACK:
                    self.tello.move_back(20)
                    self.start_time = time.time()
                    self.state = State.SEARCH

                elif self.state == State.FINISHED:
                    print("[INFO] Missão concluída. Pousando...")
                    break

        except KeyboardInterrupt:
            print("[INFO] Ctrl+C recebido dentro de run(), pousando...")

        finally:
            # garante pouso e limpeza mesmo em erro ou Ctrl+C
            self.tello.land()
            print(f"[INFO] Bateria final: {self.tello.get_battery()}%")
            with open('log_qr.pkl', 'wb') as f:
                pickle.dump(self.infos, f)
            self.tello.streamoff()
            cv2.destroyAllWindows()
            self.tello.end()

if __name__ == '__main__':
    # instância global para o handler de sinal
    qr_bot = QRAutonomy()

    # handler para Ctrl+C no main thread
    def handler(sig, frame):
        print("\n[INFO] Ctrl+C detectado! Pousando imediatamente...")
        qr_bot.tello.land()
        qr_bot.tello.streamoff()
        cv2.destroyAllWindows()
        qr_bot.tello.end()
        sys.exit(0)

    signal.signal(signal.SIGINT, handler)

    # inicia a missão
    qr_bot.run()
