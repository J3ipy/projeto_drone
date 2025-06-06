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
    next_qr_id = 2 
    running = True
    
    search_start_time = None
    search_direction = "right"
    sweep_count = 0
    last_command_time = 0
    command_to_execute = None

    try:
        while running:
            current_time = time.time()
            frame = frame_reader.frame
            frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
            frame_center_x = config.FRAME_WIDTH // 2
            qr_data, qr_rect = vision.detect_and_draw_qr(frame)

            if current_time - last_command_time > config.COMMAND_INTERVAL:
                if mission_state == "SEARCHING":
                    if search_start_time is None: 
                        search_start_time = time.time()
                    
                    if search_direction == "right": 
                        tello.send_rc_control(config.SEARCH_SWEEP_SPEED, 0, 0, 0)
                    else: 
                        tello.send_rc_control(-config.SEARCH_SWEEP_SPEED, 0, 0, 0)
                    
                    if qr_data:
                        qr_id, _, _ = vision.parse_command(qr_data)
                        if qr_id == next_qr_id:
                            print(f"QR ID {next_qr_id} encontrado! Navegando...")
                            tello.send_rc_control(0, 0, 0, 0)
                            mission_state = "NAVIGATING"
                            search_start_time = None 
                    
                    elif (time.time() - search_start_time) > config.SEARCH_TIMEOUT:
                        sweep_count += 1
                        search_direction = "left" if search_direction == "right" else "right"
                        print(f"Invertendo direção da varredura para: {search_direction}")
                        
                        if sweep_count >= 2:
                            print(f"Ciclo de varredura completo. Subindo {config.SEARCH_ALTITUDE_INCREMENT} cm.")
                            tello.send_rc_control(0, 0, 0, 0)
                            
                            # <<< CORREÇÃO: Pausa para estabilizar antes de subir >>>
                            time.sleep(0.5) 

                            tello.move_up(config.SEARCH_ALTITUDE_INCREMENT)
                            sweep_count = 0
                        
                        search_start_time = time.time()

                elif mission_state == "NAVIGATING":
                    if not qr_rect:
                        mission_state = "SEARCHING"
                    else:
                        x, y, w, h = qr_rect
                        qr_center_x = x + w // 2
                        qr_area = w * h
                        error = qr_center_x - frame_center_x
                        yaw_cmd = int((error / frame_center_x) * config.YAW_SPEED_CENTERING)
                        fwd_cmd = 0
                        if qr_area < config.TARGET_AREA_RANGE[0]:
                            fwd_cmd = config.FORWARD_SPEED
                        elif qr_area > config.TARGET_AREA_RANGE[1]:
                            fwd_cmd = -config.FORWARD_SPEED
                        
                        tello.send_rc_control(0, fwd_cmd, 0, yaw_cmd)

                        if abs(error) < config.CENTERING_TOLERANCE and config.TARGET_AREA_RANGE[0] <= qr_area <= config.TARGET_AREA_RANGE[1]:
                            print("Pronto para executar o comando.")
                            mission_state = "EXECUTING"
                            command_to_execute = vision.parse_command(qr_data)
                            tello.send_rc_control(0, 0, 0, 0)
                
                last_command_time = current_time

            if mission_state == "EXECUTING":
                if command_to_execute:
                    qr_id, cmd, val_str = command_to_execute
                    
                    print(f"EXECUTANDO COMANDO SALVO: ID={qr_id}, CMD={cmd}, VAL={val_str}")
                    controller.execute_command(tello, cmd, val_str)
                    
                    if cmd == 'land':
                        mission_state = "DONE"
                        running = False
                    else:
                        next_qr_id += 1
                        mission_state = "SEARCHING"
                    
                    command_to_execute = None
                else:
                    print("ERRO: Estado EXECUTING sem comando salvo. Voltando a procurar.")
                    mission_state = "SEARCHING"

            display.draw_flight_data(frame, tello)
            display.draw_mission_status(frame, mission_state, next_qr_id)
            cv2.imshow("Tello Mission Control", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                print("!!! PARADA DE EMERGÊNCIA ACIONADA !!!")
                tello.send_rc_control(0, 0, 0, 0)

    finally:
        print("Encerrando a missão e pousando...")
        tello.land()
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()
        print("Recursos liberados.")

if __name__ == "__main__":
    main()