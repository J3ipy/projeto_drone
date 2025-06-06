# display_module.py

import cv2

def draw_flight_data(frame, drone):
    """Desenha os dados de voo e status do drone no frame de vídeo."""
    
    # Cores e fontes
    TEXT_COLOR = (255, 255, 255) # Branco
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 1

    # Posição inicial do texto
    y_pos = 25
    
    # Dados a serem exibidos
    flight_data = {
        "Bateria": f"{drone.get_battery()}%",
        "Altura": f"{drone.get_height()} cm",
        "Tempo de Voo": f"{drone.get_flight_time()} s",
        "Temperatura": f"{drone.get_temperature()} C",
        "TOF": f"{drone.get_distance_tof()} cm",
        "Pitch": f"{drone.get_pitch()}°",
        "Roll": f"{drone.get_roll()}°",
        "Yaw": f"{drone.get_yaw()}°"
    }
    
    for key, value in flight_data.items():
        text = f"{key}: {value}"
        cv2.putText(frame, text, (10, y_pos), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
        y_pos += 25 # Move para a próxima linha

def draw_mission_status(frame, state, next_id):
    """Desenha o status atual da missão no frame."""
    
    # Cores e fontes
    STATE_COLOR = (0, 255, 255) # Amarelo
    FONT = cv2.FONT_HERSHEY_DUPLEX
    FONT_SCALE = 0.8
    FONT_THICKNESS = 2
    
    status_text = f"ESTADO: {state}"
    id_text = f"PROXIMO QR ID: {next_id}"

    # Desenha na parte inferior do frame
    frame_height = frame.shape[0]
    cv2.putText(frame, status_text, (10, frame_height - 40), FONT, FONT_SCALE, STATE_COLOR, FONT_THICKNESS, cv2.LINE_AA)
    cv2.putText(frame, id_text, (10, frame_height - 15), FONT, FONT_SCALE, STATE_COLOR, FONT_THICKNESS, cv2.LINE_AA)