# main.py

import cv2
import time
import pickle # <<< ADICIONE ESTE IMPORT
from datetime import datetime # <<< ADICIONE ESTE IMPORT
from djitellopy import Tello
import config
import vision_module as vision
import display_module as display
from mission_controller import MissionController

def main():
    # --- Inicialização ---
    tello = Tello()
    tello.connect()
    tello.streamon()
    frame_reader = tello.get_frame_read()
    print(f"Bateria Inicial: {tello.get_battery()}%")

    # --- Coleta de Dados ---
    flight_log = [] # <<< ADICIONE ESTA LINHA

    # --- Cria e inicia o controlador da missão ---
    mission = MissionController(tello)
    mission.start()

    start_time = time.time() # <<< ADICIONE ESTA LINHA

    try:
        while mission.running:
            
            if tello.get_battery() < config.MINIMUM_BATTERY:
                print(f"!!! BATERIA CRÍTICA ({tello.get_battery()}%) !!! ABORTANDO MISSÃO E POUSANDO.")
                mission.running = False
                continue
            
            frame = frame_reader.frame
            frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
            qr_data, qr_rect = vision.detect_and_draw_qr(frame)
            
            mission.update(qr_data, qr_rect)
            
            # <<< ADICIONE ESTE BLOCO PARA COLETAR DADOS A CADA FRAME >>>
            log_entry = {
                'timestamp': time.time() - start_time,
                'state': mission.mission_state,
                'target_qr': mission.target_qr,
                'battery': tello.get_battery(),
                'height': tello.get_height(),
                'pitch': tello.get_pitch(),
                'roll': tello.get_roll(),
                'yaw': tello.get_yaw()
            }
            flight_log.append(log_entry)
            # <<< FIM DO BLOCO DE COLETA >>>

            display.draw_flight_data(frame, tello)
            display.draw_mission_status(frame, mission.mission_state, mission.target_qr)
            cv2.imshow("Tello Mission Control", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                mission.stop()
            elif key == ord(' '):
                tello.send_rc_control(0, 0, 0, 0)

    finally:
        print("Encerrando a missão...")
        
        # <<< ADICIONE ESTE BLOCO PARA SALVAR OS DADOS >>>
        filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pkl"
        with open(filename, 'wb') as f:
            pickle.dump(flight_log, f)
        print(f"Log de voo salvo em: {filename}")
        # <<< FIM DO BLOCO DE SALVAMENTO >>>

        if mission.command_thread.is_alive():
            mission.stop()
            mission.command_thread.join()

        tello.land()
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()
        print("Recursos liberados.")

if __name__ == "__main__":
    main()