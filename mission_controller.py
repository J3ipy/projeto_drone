# mission_controller.py

import time
import threading
import queue
import json # <<< ADICIONADO: Importa a biblioteca para ler o arquivo JSON
from djitellopy import Tello
import config
import drone_module as controller

class MissionController:
    def __init__(self, tello: Tello):
        self.tello = tello
        # <<< MUDANÇA: Agora o plano e a sequência são carregados pelo método _get_mission_plan >>>
        self.mission_plan = {}
        self.mission_sequence = []
        self._get_mission_plan()

        self.mission_state = "IDLE"
        self.running = True
        self.current_step_index = 0
        self.last_rc_time = 0
        self.target_lost_time = None
        self.search_start_time = None
        self.search_direction = "right"
        self.sweep_count = 0
        self.command_queue = queue.Queue()
        self.state_lock = threading.Lock()
        self.command_thread = threading.Thread(target=self._command_worker)
        self.command_thread.daemon = True
        self.command_thread.start()

    # <<< MUDANÇA: Método reescrito para carregar o plano de um arquivo externo >>>
    def _get_mission_plan(self):
        """Carrega o plano de voo e a sequência de um arquivo mission.json."""
        try:
            with open('mission.json', 'r') as f:
                mission_data = json.load(f)
                self.mission_plan = mission_data['plan']
                self.mission_sequence = mission_data['sequence']
                print("Plano de missão carregado com sucesso de 'mission.json'.")
        except FileNotFoundError:
            print("ERRO: Arquivo 'mission.json' não encontrado! Usando plano de emergência (pousar).")
            # Define um plano de emergência seguro caso o arquivo não exista
            self.mission_plan = {'A': {'action': 'land', 'value': '0'}}
            self.mission_sequence = ['A']
        except KeyError:
            print("ERRO: Arquivo 'mission.json' está mal formatado! Faltam as chaves 'sequence' ou 'plan'.")
            self.mission_plan = {'A': {'action': 'land', 'value': '0'}}
            self.mission_sequence = ['A']

    
    @property
    def target_qr(self) -> str:
        if self.current_step_index < len(self.mission_sequence):
            return self.mission_sequence[self.current_step_index]
        return "DONE"

    def start(self):
        print("Iniciando a missão... Enviando comando de decolagem.")
        self.command_queue.put(('takeoff', '0'))
        self.mission_state = "WAITING"

    def stop(self):
        self.running = False
        self.command_queue.put(('exit', None))

    def _command_worker(self):
        while self.running:
            try:
                action, value = self.command_queue.get(timeout=1)
                if action == 'exit': break
                
                print(f"[WORKER THREAD] Executando: {action}...")
                controller.execute_command(self.tello, action, value)
                print(f"[WORKER THREAD] ...Comando {action} concluído.")
                
                self._update_state_after_command(action)
            except queue.Empty:
                continue
    
    def _update_state_after_command(self, completed_action: str):
        """Atualiza o estado da missão de forma segura após um comando ser concluído."""
        with self.state_lock:
            if completed_action == 'land':
                self.mission_state = "DONE"
                self.running = False
                return

            if completed_action == 'go':
                self.current_step_index += 1
            
            if self.current_step_index >= len(self.mission_sequence):
                print("[WORKER THREAD] Missão concluída! Enviando comando para pousar.")
                self.command_queue.put(('land', '0'))
                self.mission_state = "WAITING"
            else:
                print(f"[WORKER THREAD] Próximo estado: SEARCHING para o alvo '{self.target_qr}'")
                self.mission_state = "SEARCHING"

    def update(self, qr_data, qr_rect):
        """Chamado a cada frame para lidar com movimentos contínuos."""
        if self.mission_state in ["SEARCHING", "NAVIGATING"]:
            self._handle_rc_movement(qr_data, qr_rect)

    def _handle_rc_movement(self, qr_data, qr_rect):
        """Lida com os movimentos de RC (SEARCHING e NAVIGATING)."""
        if time.time() - self.last_rc_time > config.COMMAND_INTERVAL:
            if self.mission_state == "SEARCHING":
                if self.search_start_time is None: self.search_start_time = time.time()
                
                speed = config.SEARCH_SWEEP_SPEED if self.search_direction == "right" else -config.SEARCH_SWEEP_SPEED
                self.tello.send_rc_control(speed, 0, 0, 0)
                
                if qr_data and qr_data == self.target_qr:
                    self.tello.send_rc_control(0, 0, 0, 0)
                    with self.state_lock: self.mission_state = "NAVIGATING"
                    self.search_start_time = None
                
                elif (time.time() - self.search_start_time) > config.SEARCH_TIMEOUT:
                    self.sweep_count += 1
                    self.search_direction = "left" if self.search_direction == "right" else "right"
                    if self.sweep_count >= 2:
                        print("[MAIN THREAD] Ciclo de busca completo. Enviando comando para subir...")
                        self.tello.send_rc_control(0, 0, 0, 0)
                        self.command_queue.put(('move_up', str(config.SEARCH_ALTITUDE_INCREMENT)))
                        self.sweep_count = 0
                        with self.state_lock: self.mission_state = "WAITING"
                    self.search_start_time = time.time()
            
            elif self.mission_state == "NAVIGATING":
                if qr_rect:
                    self.target_lost_time = None
                    x, y, w, h = qr_rect
                    error = (x + w // 2) - (config.FRAME_WIDTH // 2)
                    left_right_cmd = int((error / (config.FRAME_WIDTH // 2)) * config.STRAFE_SPEED_CENTERING)
                    qr_area = w * h
                    fwd_cmd = 0
                    if qr_area < config.TARGET_AREA_RANGE[0]: fwd_cmd = config.FORWARD_SPEED
                    elif qr_area > config.TARGET_AREA_RANGE[1]: fwd_cmd = -config.FORWARD_SPEED
                    self.tello.send_rc_control(left_right_cmd, fwd_cmd, 0, 0)

                    if abs(error) < config.CENTERING_TOLERANCE and config.TARGET_AREA_RANGE[0] <= qr_area <= config.TARGET_AREA_RANGE[1]:
                        print(f"[MAIN THREAD] Alvo '{self.target_qr}' centralizado. Enviando comando para o worker...")
                        self.tello.send_rc_control(0, 0, 0, 0)
                        mission_step = self.mission_plan[self.target_qr]
                        self.command_queue.put((mission_step['action'], mission_step['value']))
                        with self.state_lock: self.mission_state = "WAITING"
                else:
                    # Se o alvo for perdido durante a navegação, usa o período de graça
                    self.tello.send_rc_control(0, 0, 0, 0)
                    if self.target_lost_time is None: self.target_lost_time = time.time()
                    elif (time.time() - self.target_lost_time) > config.NAV_GRACE_PERIOD:
                        with self.state_lock: self.mission_state = "SEARCHING"

            self.last_rc_time = time.time()