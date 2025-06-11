# display_module.py
import cv2

def draw_flight_data(frame, drone):
    # Esta função continua a mesma...
    TEXT_COLOR = (255, 255, 255)
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    y_pos = 25
    flight_data = {
        "Bateria": f"{drone.get_battery()}%",
        "Altura": f"{drone.get_height()} cm",
        "Pitch": f"{drone.get_pitch()} deg",
        "Roll": f"{drone.get_roll()} deg",
    }
    for key, value in flight_data.items():
        text = f"{key}: {value}"
        # <<< MELHORIA: Bateria com cores para alerta visual >>>
        color = TEXT_COLOR
        if key == "Bateria":
            battery_val = drone.get_battery()
            if battery_val <= 20:
                color = (0, 0, 255) # Vermelho
            elif battery_val <= 50:
                color = (0, 255, 255) # Amarelo
        cv2.putText(frame, text, (10, y_pos), FONT, FONT_SCALE, color, 2)
        y_pos += 25

def draw_mission_status(frame, state, target_qr, qr_area=0, error=0):
    """Desenha o status da missão, incluindo dados de navegação."""
    STATE_COLOR = (0, 255, 255) # Amarelo
    FONT = cv2.FONT_HERSHEY_DUPLEX
    FONT_SCALE = 0.7
    frame_height = frame.shape[0]

    # Status da missão
    status_text = f"ESTADO: {state}"
    target_text = f"ALVO ATUAL: {target_qr}"
    cv2.putText(frame, status_text, (10, frame_height - 65), FONT, FONT_SCALE, STATE_COLOR, 1)
    cv2.putText(frame, target_text, (10, frame_height - 40), FONT, FONT_SCALE, STATE_COLOR, 1)

    # <<< MELHORIA: Mostra dados de navegação em tempo real >>>
    if state == "NAVIGATING":
        nav_text = f"Area: {qr_area} | Erro: {error}px"
        cv2.putText(frame, nav_text, (10, frame_height - 15), FONT, FONT_SCALE, (255, 100, 100), 1)