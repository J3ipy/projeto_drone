# --- Parâmetros da Busca Inteligente ---
# Tempo em segundos para procurar em uma altitude antes de subir
SEARCH_TIMEOUT = 2

# Distância em cm para subir se o QR code não for encontrado
SEARCH_ALTITUDE_INCREMENT = 20

# --- Configurações da Câmera e Janela ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# Velocidade do movimento lateral durante a busca (cm/s)
SEARCH_SWEEP_SPEED = 15

# Velocidade para frente/trás ao se aproximar de um QR code
FORWARD_SPEED = 20

# A "área" do QR code (largura * altura) que consideramos ideal para leitura
# O drone tentará se aproximar até que a área do QR esteja neste intervalo.
TARGET_AREA_RANGE = (14000, 17000)

# Velocidade para os comandos 'go_xyz_speed' em cm/s
GO_XYZ_SPEED = 25

# Quão perto (em pixels) o centro do QR deve estar do centro do frame para ser considerado "centralizado"
CENTERING_TOLERANCE = 20

COMMAND_INTERVAL = 0.05 