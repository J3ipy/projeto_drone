# robotic_pathfinding_v4.py

import pygame
import time
import random
import math
from collections import deque
import heapq

# --- CONFIGURAÇÃO DA VISUALIZAÇÃO (PYGAME) ---
# Cores
COR_FUNDO = (255, 255, 255)
COR_LINHA = (200, 200, 200)
COR_OBSTACULO = (0, 0, 0)
COR_INICIO = (0, 255, 0)
COR_FIM = (255, 0, 0)
COR_CAMINHO = (0, 0, 255)
COR_EXPLORADO = (255, 165, 0)
COR_FRONTEIRA = (173, 216, 230)
COR_ARVORE_RRT = (150, 0, 150) # Roxo para a árvore RRT

# Dimensões e Controle de Frame Rate
LARGURA_JANELA = 800
NUM_CELULAS = 50
TAMANHO_CELULA = LARGURA_JANELA // NUM_CELULAS
FPS = 60

# Parâmetros para RRT/RRT*
RRT_ITERATIONS = 5000
RRT_STEP_SIZE = 2  # Em número de células

# --- CLASSE NODE ---
class Node:
    """ Representa uma célula na grade (um vértice no grafo). """
    def __init__(self, row, col):
        self.row, self.col = row, col
        # Coordenadas em pixels do centro da célula
        self.x_pixel = col * TAMANHO_CELULA + TAMANHO_CELULA // 2
        self.y_pixel = row * TAMANHO_CELULA + TAMANHO_CELULA // 2
        self.is_obstacle = False
        self.neighbors = []
        self.parent = None
        self.color = COR_FUNDO
        # Atributos para os algoritmos
        self.g_cost = float('inf')
        self.h_cost = float('inf')
        self.f_cost = float('inf')

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def draw(self, window):
        pygame.draw.rect(window, self.color, (self.col * TAMANHO_CELULA, self.row * TAMANHO_CELULA, TAMANHO_CELULA, TAMANHO_CELULA))

    def update_neighbors(self, grid_nodes):
        self.neighbors = []
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for move_r, move_c in moves:
            nr, nc = self.row + move_r, self.col + move_c
            if 0 <= nr < NUM_CELULAS and 0 <= nc < NUM_CELULAS and not grid_nodes[nr][nc].is_obstacle:
                self.neighbors.append(grid_nodes[nr][nc])

# --- ALGORITMOS DE BUSCA EM GRADE ---
def reconstruct_path(end_node, draw_update_func, is_rrt=False):
    current = end_node
    while current.parent is not None:
        if not is_rrt:
            # Colore o caminho para algoritmos de grade
            current.parent.color = COR_CAMINHO
        else:
            # Desenha a linha do caminho para RRT
            pygame.draw.line(pygame.display.get_surface(), COR_CAMINHO, (current.x_pixel, current.y_pixel), (current.parent.x_pixel, current.parent.y_pixel), 4)
        current = current.parent
        draw_update_func()
        pygame.time.delay(20)
    # Restaura a cor do nó inicial
    if current: current.color = COR_INICIO
    draw_update_func()


def bfs_search(start_node, end_node, draw_update_func):
    queue = deque([start_node]); visited = {start_node}; start_node.parent = None
    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return False
        current_node = queue.popleft()
        if current_node == end_node: reconstruct_path(end_node, draw_update_func); return True
        if current_node != start_node: current_node.color = COR_EXPLORADO
        for neighbor in current_node.neighbors:
            if neighbor not in visited:
                visited.add(neighbor); neighbor.parent = current_node
                neighbor.color = COR_FRONTEIRA; queue.append(neighbor)
        draw_update_func(); pygame.time.delay(1)
    return False

def dijkstra_search(start_node, end_node, draw_update_func):
    start_node.g_cost = 0; pq = [(0, start_node)]; visited = set()
    while pq:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return False
        current_cost, current_node = heapq.heappop(pq)
        if current_node in visited: continue
        visited.add(current_node)
        if current_node == end_node: reconstruct_path(end_node, draw_update_func); return True
        if current_node != start_node: current_node.color = COR_EXPLORADO
        for neighbor in current_node.neighbors:
            new_cost = current_node.g_cost + 1
            if new_cost < neighbor.g_cost:
                neighbor.g_cost = new_cost; neighbor.parent = current_node
                neighbor.color = COR_FRONTEIRA; heapq.heappush(pq, (new_cost, neighbor))
        draw_update_func(); pygame.time.delay(1)
    return False

def heuristic(node_a, node_b):
    return abs(node_a.row - node_b.row) + abs(node_a.col - node_b.col)

def a_star_search(start_node, end_node, draw_update_func):
    start_node.g_cost = 0; start_node.h_cost = heuristic(start_node, end_node)
    start_node.f_cost = start_node.g_cost + start_node.h_cost
    pq = [(start_node.f_cost, start_node)]; visited = {start_node}
    while pq:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return False
        current_f_cost, current_node = heapq.heappop(pq)
        if current_node == end_node: reconstruct_path(end_node, draw_update_func); return True
        if current_node != start_node: current_node.color = COR_EXPLORADO
        for neighbor in current_node.neighbors:
            new_g_cost = current_node.g_cost + 1
            if new_g_cost < neighbor.g_cost:
                if neighbor not in visited:
                    visited.add(neighbor); neighbor.parent = current_node
                    neighbor.g_cost = new_g_cost; neighbor.h_cost = heuristic(neighbor, end_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    neighbor.color = COR_FRONTEIRA; heapq.heappush(pq, (neighbor.f_cost, neighbor))
        draw_update_func(); pygame.time.delay(1)
    return False

# --- NOVOS ALGORITMOS: RRT e RRT* ---
def get_distance(node1, node2):
    return math.sqrt((node1.row - node2.row)**2 + (node1.col - node2.col)**2)

def get_random_node(grid_nodes, end_node):
    if random.randint(0, 100) > 90:
        return end_node
    while True:
        rand_row = random.randint(0, NUM_CELULAS - 1)
        rand_col = random.randint(0, NUM_CELULAS - 1)
        if not grid_nodes[rand_row][rand_col].is_obstacle:
            return grid_nodes[rand_row][rand_col]

def find_nearest_node(tree_nodes, random_node):
    return min(tree_nodes, key=lambda node: get_distance(node, random_node))

def is_collision_free(from_node, to_node, grid_nodes):
    dist = max(abs(from_node.col - to_node.col), abs(from_node.row - to_node.row))
    if dist == 0: return True
    for i in range(int(dist) + 1):
        t = i / dist
        col = round(from_node.col + t * (to_node.col - from_node.col))
        row = round(from_node.row + t * (to_node.row - from_node.row))
        if grid_nodes[row][col].is_obstacle:
            return False
    return True

def steer(from_node, to_node, grid_nodes):
    dist = get_distance(from_node, to_node)
    if dist < RRT_STEP_SIZE:
        if is_collision_free(from_node, to_node, grid_nodes):
            return to_node
        return None

    ratio = RRT_STEP_SIZE / dist
    new_row = int(from_node.row + (to_node.row - from_node.row) * ratio)
    new_col = int(from_node.col + (to_node.col - from_node.col) * ratio)
    
    if 0 <= new_row < NUM_CELULAS and 0 <= new_col < NUM_CELULAS:
        new_node = grid_nodes[new_row][new_col]
        if not new_node.is_obstacle and is_collision_free(from_node, new_node, grid_nodes):
            return new_node
    return None

def rrt_search(start_node, end_node, grid_nodes, draw_func):
    tree = {start_node}
    start_node.parent = None

    for i in range(RRT_ITERATIONS):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return False
            
        rand_node = get_random_node(grid_nodes, end_node)
        nearest_node = find_nearest_node(tree, rand_node)
        new_node = steer(nearest_node, rand_node, grid_nodes)

        if new_node and new_node not in tree:
            new_node.parent = nearest_node
            tree.add(new_node)
            
            if get_distance(new_node, end_node) <= RRT_STEP_SIZE and is_collision_free(new_node, end_node, grid_nodes):
                end_node.parent = new_node
                draw_func(tree)
                reconstruct_path(end_node, lambda: draw_func(tree), is_rrt=True)
                return True
        
        if i % 20 == 0:
            draw_func(tree)

    return False

def rrt_star_search(start_node, end_node, grid_nodes, draw_func):
    tree = {start_node}
    start_node.g_cost = 0
    start_node.parent = None
    
    for i in range(RRT_ITERATIONS):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return False

        rand_node = get_random_node(grid_nodes, end_node)
        nearest_node = find_nearest_node(tree, rand_node)
        new_node = steer(nearest_node, rand_node, grid_nodes)

        if new_node and new_node not in tree:
            radius = RRT_STEP_SIZE * 2
            near_nodes = [n for n in tree if get_distance(n, new_node) <= radius]
            best_parent = nearest_node
            min_cost = nearest_node.g_cost + get_distance(nearest_node, new_node)

            for near_node in near_nodes:
                cost = near_node.g_cost + get_distance(near_node, new_node)
                if cost < min_cost and is_collision_free(near_node, new_node, grid_nodes):
                    min_cost = cost
                    best_parent = near_node

            new_node.parent = best_parent
            new_node.g_cost = min_cost
            tree.add(new_node)
            
            for near_node in near_nodes:
                cost_via_new = new_node.g_cost + get_distance(new_node, near_node)
                if cost_via_new < near_node.g_cost and is_collision_free(new_node, near_node, grid_nodes):
                    near_node.parent = new_node
                    near_node.g_cost = cost_via_new
            
            if get_distance(new_node, end_node) <= RRT_STEP_SIZE and is_collision_free(new_node, end_node, grid_nodes):
                end_node.parent = new_node
                draw_func(tree)
                reconstruct_path(end_node, lambda: draw_func(tree), is_rrt=True)
                return True

        if i % 20 == 0:
            draw_func(tree)

    return False

# --- FUNÇÕES DE CONTROLE E DESENHO ---
def create_grid():
    return [[Node(r, c) for c in range(NUM_CELULAS)] for r in range(NUM_CELULAS)]

def draw(window, grid_nodes, rrt_tree=None):
    window.fill(COR_FUNDO)
    for row in grid_nodes:
        for node in row:
            node.draw(window)
    
    if rrt_tree:
        for node in rrt_tree:
            if node.parent:
                pygame.draw.line(window, COR_ARVORE_RRT, (node.x_pixel, node.y_pixel), (node.parent.x_pixel, node.parent.y_pixel), 2)

    for i in range(NUM_CELULAS):
        pygame.draw.line(window, COR_LINHA, (0, i * TAMANHO_CELULA), (LARGURA_JANELA, i * TAMANHO_CELULA))
        pygame.draw.line(window, COR_LINHA, (i * TAMANHO_CELULA, 0), (i * TAMANHO_CELULA, LARGURA_JANELA))
    
    pygame.display.update()

def get_clicked_pos(pos):
    # **CORREÇÃO AQUI**
    x, y = pos
    row = y // TAMANHO_CELULA
    col = x // TAMANHO_CELULA
    return row, col

def reset_board(grid_nodes):
    for row in grid_nodes:
        for node in row:
            node.parent = None; node.g_cost = float('inf'); node.h_cost = float('inf'); node.f_cost = float('inf')
            if not node.is_obstacle: node.color = COR_FUNDO

# --- FUNÇÃO PRINCIPAL ---
def main(window):
    clock = pygame.time.Clock()
    grid_nodes = create_grid()
    start_node, end_node = None, None
    
    running = True
    while running:
        clock.tick(FPS)
        draw(window, grid_nodes)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos)
                if 0 <= row < NUM_CELULAS and 0 <= col < NUM_CELULAS:
                    node = grid_nodes[row][col]
                    if event.button == 1: 
                        if not start_node and node != end_node: start_node = node; start_node.color = COR_INICIO
                        elif not end_node and node != start_node: end_node = node; end_node.color = COR_FIM
                        elif node != start_node and node != end_node: node.is_obstacle = True; node.color = COR_OBSTACULO
                    elif event.button == 3:
                        node.is_obstacle = False; node.color = COR_FUNDO
                        if node == start_node: start_node = None
                        if node == end_node: end_node = None
            
            if pygame.mouse.get_pressed()[0]:
                if start_node and end_node:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos)
                    if 0 <= row < NUM_CELULAS and 0 <= col < NUM_CELULAS:
                        node = grid_nodes[row][col]
                        if node != start_node and node != end_node: node.is_obstacle = True; node.color = COR_OBSTACULO

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    start_node, end_node = None, None
                    grid_nodes = create_grid()

                if start_node and end_node:
                    reset_board(grid_nodes)
                    start_node.color = COR_INICIO; end_node.color = COR_FIM
                    
                    grid_algos = {pygame.K_b: (bfs_search, "BFS"), pygame.K_d: (dijkstra_search, "Dijkstra"), pygame.K_a: (a_star_search, "A*")}
                    rrt_algos = {pygame.K_r: (rrt_search, "RRT"), pygame.K_t: (rrt_star_search, "RRT*")}

                    run_algorithm = None
                    is_rrt_algo = False

                    if event.key in grid_algos:
                        for row in grid_nodes:
                            for node in row: node.update_neighbors(grid_nodes)
                        run_algorithm, algo_name = grid_algos[event.key]
                    
                    elif event.key in rrt_algos:
                        run_algorithm, algo_name = rrt_algos[event.key]
                        is_rrt_algo = True

                    if run_algorithm:
                        print(f"\n--- Iniciando Algoritmo: {algo_name} ---")
                        start_time = time.time()
                        
                        if is_rrt_algo:
                            path_found = run_algorithm(start_node, end_node, grid_nodes, lambda tree=None: draw(window, grid_nodes, tree))
                        else:
                            path_found = run_algorithm(start_node, end_node, lambda: draw(window, grid_nodes))
                        
                        end_time = time.time()
                        print(f"Tempo de execução: {end_time - start_time:.4f} segundos")

                        if path_found:
                            path_len = 0; curr = end_node
                            while curr and curr.parent is not None:
                                path_len += 1; curr = curr.parent
                            print(f"Comprimento do caminho: {path_len} passos")
                        else:
                            print("Nenhum caminho foi encontrado.")
    pygame.quit()

# Ponto de entrada do programa
if __name__ == "__main__":
    pygame.init()
    window = pygame.display.set_mode((LARGURA_JANELA, LARGURA_JANELA))
    pygame.display.set_caption("Planejamento de Caminho | Grade (B,D,A) | Amostragem (R,T) | Limpar (C)")
    main(window)