# debug_vision.py

import cv2
from djitellopy import Tello

# Importa os módulos que já criamos
import config
import vision_module as vision
import display_module as display

def main():
    """
    Script de depuração para testar a detecção de QR codes sem executar a lógica de voo.
    O drone permanecerá parado (no chão ou pairando) enquanto você testa a visão.
    """
    # --- Inicialização ---
    tello = Tello()
    tello.connect()
    tello.streamon()
    frame_reader = tello.get_frame_read()
    
    print("--- MODO DE DEBUG DE VISÃO ATIVADO ---")
    print(f"Bateria: {tello.get_battery()}%")
    print("\nInstruções:")
    print(" - O drone ficará parado.")
    print(" - Aponte um QR code da missão para a câmera para testar a detecção.")
    print(" - Pressione 'H' para decolar e pairar (hover).")
    print(" - Pressione 'L' para pousar se estiver voando.")
    print(" - Pressione 'Q' para sair do programa.")
    
    try:
        while True:
            # --- Leitura e Processamento do Frame ---
            frame = frame_reader.frame
            frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))

            # --- Teste do Módulo de Visão ---
            # Tenta detectar e desenhar o QR code no frame
            qr_data, qr_rect = vision.detect_and_draw_qr(frame)
            
            # Se um QR code for detectado, processa e exibe as informações
            if qr_data:
                # Tenta analisar os dados do QR code
                qr_id, cmd, val_str = vision.parse_command(qr_data)
                
                if qr_id is not None:
                    # Imprime no console para fácil depuração
                    print(f"QR Detectado -> ID: {qr_id}, Comando: {cmd}, Valor: {val_str}")
                    
                    # Desenha as informações analisadas no frame
                    info_text = f"ID:{qr_id} | CMD:{cmd} | VAL:{val_str}"
                    cv2.putText(frame, info_text, (10, frame.shape[0] - 15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # --- Desenhar HUD ---
            # Usa o display_module para mostrar os dados de voo
            display.draw_flight_data(frame, tello)
            
            # Adiciona um texto indicando o modo de debug
            cv2.putText(frame, "MODO DEBUG: VISAO", (10, frame.shape[0] - 40), 
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 255), 2)

            # --- Exibir Frame ---
            cv2.imshow("Tello Vision Debug", frame)
            
            # --- Controle Manual Básico ---
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('h'):
                print("Comando: Decolar e pairar...")
                tello.takeoff()
            elif key == ord('l'):
                print("Comando: Pousar...")
                tello.land()

    finally:
        # --- Limpeza ---
        print("Encerrando o modo de debug...")
        tello.land() # Garante o pouso por segurança
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()
        print("Recursos liberados.")

if __name__ == "__main__":
    main()