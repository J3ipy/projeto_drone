# plot_log.py

import pickle
import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_flight_data(log_file):
    """Lê um arquivo de log .pkl e gera gráficos de telemetria."""
    
    try:
        with open(log_file, 'rb') as f:
            log_data = pickle.load(f)
    except FileNotFoundError:
        print(f"Erro: Arquivo '{log_file}' não encontrado.")
        return

    if not log_data:
        print("Log de voo está vazio.")
        return

    # Converte a lista de dicionários para um DataFrame do Pandas
    df = pd.DataFrame.from_records(log_data)

    # Cria a figura e os subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
    fig.suptitle(f'Análise de Voo - {log_file}', fontsize=16)

    # --- Plot 1: Altitude e Bateria ---
    ax1.plot(df['timestamp'], df['height'], label='Altitude (cm)', color='tab:blue')
    ax1.set_ylabel('Altitude (cm)', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True)
    
    ax1b = ax1.twinx() # Cria um segundo eixo Y
    ax1b.plot(df['timestamp'], df['battery'], label='Bateria (%)', color='tab:green', linestyle='--')
    ax1b.set_ylabel('Bateria (%)', color='tab:green')
    ax1b.tick_params(axis='y', labelcolor='tab:green')
    ax1.legend(loc='upper left')
    ax1b.legend(loc='upper right')

    # --- Plot 2: Orientação ---
    ax2.plot(df['timestamp'], df['pitch'], label='Pitch (°)', alpha=0.8)
    ax2.plot(df['timestamp'], df['roll'], label='Roll (°)', alpha=0.8)
    ax2.plot(df['timestamp'], df['yaw'], label='Yaw (°)', alpha=0.8)
    ax2.set_ylabel('Orientação (°)')
    ax2.legend()
    ax2.grid(True)

    # --- Plot 3: Estado da Missão ---
    # Mapeia os estados para números para poder plotar
    state_map = {state: i for i, state in enumerate(df['state'].unique())}
    df['state_num'] = df['state'].map(state_map)
    
    ax3.step(df['timestamp'], df['state_num'], where='post', label='Estado da Missão')
    ax3.set_yticks(list(state_map.values()))
    ax3.set_yticklabels(list(state_map.keys())) # Coloca os nomes dos estados no eixo Y
    ax3.set_ylabel('Estado da Missão')
    ax3.set_xlabel('Tempo (s)')
    ax3.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Uso: python plot_log.py <caminho_para_o_arquivo.pkl>")
    else:
        plot_flight_data(sys.argv[1])