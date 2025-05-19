import pickle
import matplotlib.pyplot as plt

# Carrega o log
with open('log_qr.pkl', 'rb') as f:
    infos = pickle.load(f)

# Extrai dados
seq     = list(range(len(infos)))
heights = [h for (_, h, _, _) in infos]
times   = [t for (_, _, _, t) in infos]

# Gráfico 1: sequência × altura
plt.figure()
plt.plot(seq, heights, marker='o')
plt.xlabel('QR # na sequência')
plt.ylabel('Altura (cm)')
plt.title('Altura do drone ao capturar cada QR')
plt.grid(True)
plt.show()

# Gráfico 2: altura × tempo
plt.figure()
plt.plot(times, heights, marker='x')
plt.xlabel('Tempo desde início (s)')
plt.ylabel('Altura (cm)')
plt.title('Evolução da altura do drone ao longo do voo')
plt.grid(True)
plt.show()
