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
            # Importa a velocidade do config
            from config import GO_XYZ_SPEED
            x, y, z = map(int, value_str.split(':'))
            drone.go_xyz_speed(x, y, z, GO_XYZ_SPEED)
        except ValueError:
            print(f"ERRO: Valor inválido para o comando 'go': {value_str}")
    else:
        print(f"AVISO: Comando '{cmd}' desconhecido.")
        
    time.sleep(1) # Pausa para garantir que o comando anterior termine