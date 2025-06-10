# vision_module.py

import cv2
import numpy as np

detector = cv2.QRCodeDetector()

def detect_and_draw_qr(frame):
    # Converte para escala de cinza
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # <<< ADICIONADO: Mostra a imagem em preto e branco para depuração >>>
    # Isso nos ajuda a ver se o contraste está bom para o detector
    _, threshold_frame = cv2.threshold(gray_frame, 128, 255, cv2.THRESH_BINARY)
    cv2.imshow("Debug Threshold", threshold_frame)
    # <<< FIM DA ADIÇÃO >>>

    # Usa o frame em escala de cinza para detectar, pode ser mais robusto
    data, bbox, _ = detector.detectAndDecode(gray_frame)

    if bbox is not None and data:
        # ... (o resto da função permanece igual) ...
        points = bbox[0].astype(int)
        cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=3)
        x_min = np.min(points[:, 0])
        y_min = np.min(points[:, 1])
        x_max = np.max(points[:, 0])
        y_max = np.max(points[:, 1])
        rect = (x_min, y_min, x_max - x_min, y_max - y_min)
        cv2.putText(frame, data, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return data, rect

    return None, None