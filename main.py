# main.py

import cv2
import time
from djitellopy import Tello

# Importa nossos módulos e configurações
import config
import vision_module as vision
import display_module as display
import drone_module as controller

def main():
    # --- PLANO DE MISSÃO ---
    MISSION_SEQUENCE = ['E', 'D', 'C', 'B', 'A']
    MISSION_PLAN = {
        'E': {'action': 'go', 'value': '0:-50:65'},
        'D': {'action': 'go', 'value': '0:-53:40'},
        'C': {'action': 'go', 'value': '0:-55:-41'},
        'B': {'action': 'go', 'value': '0:-80:-40'},
        'A': {'action': 'land', 'value': '0'}
    }

    # --- Inicialização ---
    tello = Tello()
    tello.connect()
    tello.streamon()
    frame_reader = tello.get_frame_read()
    
    print(f"Bateria Inicial: {tello.get_battery()}%")
    print("Iniciando a missão... Decolando!")
    tello.takeoff()
    time.sleep(3) 

    # --- Estado Inicial da Missão ---
    mission_state = "SEARCHING"
    current_step_index = 0
    running = True
    
    search_start_time = None
    search_direction = "right"
    sweep_count = 0
    last_command_time = 0

    try:
        while running:
            current_time = time.time()
            frame = frame_reader.frame
            frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
            target_qr = MISSION_SEQUENCE[current_step_index]
            qr_data, qr_rect = vision.detect_and_draw_qr(frame)

            if current_time - last_command_time > config.COMMAND_INTERVAL:
                if mission_state == "SEARCHING":
                    if search_start_time is None: search_start_time = time.time()
                    if search_direction == "right": tello.send_rc_control(config.SEARCH_SWEEP_SPEED, 0, 0, 0)
                    else: tello.send_rc_control(-config.SEARCH_SWEEP_SPEED, 0, 0, 0)
                    if qr_data and qr_data == target_qr:
                        tello.send_rc_control(0, 0, 0, 0)
                        mission_state = "NAVIGATING"
                        search_start_time = None 
                    elif (time.time() - search_start_time) > config.SEARCH_TIMEOUT:
                        sweep_count += 1
                        search_direction = "left" if search_direction == "right" else "right"
                        if sweep_count >= 2:
                            tello.send_rc_control(0, 0, 0, 0)
                            time.sleep(0.5)
                            tello.move_up(config.SEARCH_ALTITUDE_INCREMENT)
                            sweep_count = 0
                        search_start_time = time.time()

                elif mission_state == "NAVIGATING":
                    if not qr_rect:
                        mission_state = "SEARCHING"
                    else:
                        x, y, w, h = qr_rect
                        qr_area = w * h
                        error = (x + w // 2) - (config.FRAME_WIDTH // 2)
                        left_right_cmd = int((error / (config.FRAME_WIDTH // 2)) * config.STRAFE_SPEED_CENTERING)
                        fwd_cmd = 0
                        if qr_area < config.TARGET_AREA_RANGE[0]:
                            fwd_cmd = config.FORWARD_SPEED
                        elif qr_area > config.TARGET_AREA_RANGE[1]:
                            fwd_cmd = -config.FORWARD_SPEED
                        tello.send_rc_control(left_right_cmd, fwd_cmd, 0, 0)

                        if abs(error) < config.CENTERING_TOLERANCE and config.TARGET_AREA_RANGE[0] <= qr_area <= config.TARGET_AREA_RANGE[1]:
                            print(f"Alvo '{target_qr}' centralizado. Executando ação...")
                            mission_state = "EXECUTING"
                            tello.send_rc_control(0, 0, 0, 0)
                
                last_command_time = current_time

            if mission_state == "EXECUTING":
                time.sleep(0.5)
                mission_step = MISSION_PLAN[target_qr]
                action = mission_step['action']
                value = mission_step['value']
                print(f"EXECUTANDO AÇÃO: {action} com valor {value}")
                controller.execute_command(tello, action, value)
                if action == 'land':
                    running = False
                else:
                    current_step_index += 1
                    if current_step_index >= len(MISSION_SEQUENCE):
                        print("Último passo concluído! Pousando...")
                        tello.land()
                        running = False
                    else:
                        mission_state = "SEARCHING"
            
            display.draw_flight_data(frame, tello)
            display.draw_mission_status(frame, mission_state, target_qr)
            cv2.imshow("Tello Mission Control", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord(' '): tello.send_rc_control(0, 0, 0, 0)

    finally:
        print("Encerrando a missão...")
        tello.land()
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()