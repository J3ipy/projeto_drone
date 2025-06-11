# drone_module.py

import time

def execute_command(drone, cmd, value_str):
    """Interpreta e executa um comando no drone."""
    print(f"EXECUTANDO COMANDO: {cmd} com valor '{value_str}'")
    
    if cmd == 'takeoff':
        drone.takeoff()
    elif cmd == 'land':
        drone.land()
    elif cmd == 'go':
        try:
            from config import GO_XYZ_SPEED
            params = list(map(int, value_str.split(':')))
            drone.go_xyz_speed(params[0], params[1], params[2], GO_XYZ_SPEED)
        except ValueError:
            print(f"Erro: valor inválido para o comando 'go': {value_str}")
    
    # <<< ADICIONE ESTE BLOCO >>>
    elif cmd == 'move_up':
        try:
            drone.move_up(int(value_str))
        except ValueError:
            print(f"Erro: valor inválido para o comando 'move_up': {value_str}")

    else:
        print(f"AVISO: Comando '{cmd}' desconhecido.")
        
    time.sleep(0.5) # Pequena pausa após o comando