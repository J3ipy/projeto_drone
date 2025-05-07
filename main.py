import time
import numpy as np
from djitellopy import Tello

# Posição dos waypoints [x, y, z] em metros
path = np.array([
    [0,   0,  0.5], #subir OK

    [0,  -0.7,  0.5], # primeira diagonal direita OK
    [0,  -0.7,  0.5], # primeira diagonal direita OK

    [0,  -0.7,  -0.5], # segunda diagonal direita OK 
    [0,  -0.7,  -0.5], # segunda diagonal direira OK

    [0,  0.7,  0.5], # volta da segunda diagonal direita OK
    [0, 0.7,  0.5], # volta da segunda diagonal direita OK

    [0,  0.7,  -0.5], # volta da primeira diagonal direita OK
    [0,  0.7,  -0.5], # volta da primeira diagonal direita OK

    [0,   0,  -0.5], #pousar OK
])

# Conectar e entrar em modo SDK
tello = Tello()
tello.connect()
print(f"Bateria inicial: {tello.get_battery()}%") # verifica a bateria atual

# Decolar
tello.takeoff()
time.sleep(2)

# Seguir o path
for idx, waypoint in enumerate(path, start=1):
    # converter para centímetros e garantir tipo int
    dx, dy, dz = map(int, waypoint * 100) # converte metros em cm (O SDK do tello trabalha em cm)
    print(f"Waypoint {idx}: go {dx} {dy} {dz} at speed 20") # move em direção relativa e com uma velocidade de 20 cm/s
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
