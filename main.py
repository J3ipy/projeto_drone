import time
import numpy as np
from djitellopy import Tello

# Posição dos waypoints [x, y, z] em metros
path = np.array([
    [0,   0,  -0.4],
    [0,   0,  -0.4],
    [0,   0,  -0.4],
    [0,   0.6,  0],
    [0,   0,   0.4],
    [0,   0,   0.4],
    [0,  -0.6,  0.4]
])

# Conectar e entrar em modo SDK
tello = Tello()
tello.connect()
print(f"Bateria inicial: {tello.get_battery()}%")

# Decolar
tello.takeoff()
time.sleep(2)

# Seguir o path
for idx, waypoint in enumerate(path, start=1):
    # converter para centímetros
    dx, dy, dz = (waypoint * 100).astype(int)
    print(f"Waypoint {idx}: go {dx} {dy} {dz} at speed 20")
    # Comando 'go x y z speed' do SDK 2.0 :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}
    tello.go_xyz_speed(dx, dy, dz, 20)
    time.sleep(2)  # pausa para estabilizar

# Pousar
tello.land()
bat_final = tello.get_battery()
bat_dif = tello.get_battery() - bat_final

print(f"Bateria final: {bat_final}%")
print(f"Gasto de bateria: {bat_dif}%")

# Encerrar conexão
tello.end()
