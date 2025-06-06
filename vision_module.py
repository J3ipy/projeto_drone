# vision_module.py

import cv2
from pyzbar import pyzbar

def detect_and_draw_qr(frame):
    """
    Detecta, decodifica e desenha a caixa ao redor do primeiro QR code encontrado.
    Retorna os dados decodificados e o retângulo da posição, se encontrado.
    """
    qrcodes = pyzbar.decode(frame)
    if not qrcodes:
        return None, None

    # Pega o primeiro QR code detectado
    qr = qrcodes[0]
    data = qr.data.decode('utf-8')
    rect = qr.rect  # (x, y, w, h)

    # Desenha o retângulo e o texto no frame
    x, y, w, h = rect
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return data, rect

def parse_command(data):
    """Analisa a string de dados do QR code."""
    try:
        parts = data.split(';')
        qr_info = {}
        for part in parts:
            key, value = part.split(':', 1)
            qr_info[key] = value

        qr_id = int(qr_info['id'])
        cmd = qr_info['cmd']
        value_str = qr_info.get('value', '0')
        
        return qr_id, cmd, value_str
    except (ValueError, IndexError):
        # Retorna None se o formato do QR code for inválido
        return None, None, None