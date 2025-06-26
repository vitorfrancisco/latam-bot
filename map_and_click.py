import struct
import numpy as np
import matplotlib.pyplot as plt
import psutil
import pymem
import time
import heapq
import keyboard
import pygetwindow as gw
import serial
from controllers.mouse_controller import move_mouse_and_click, get_mouse_position, only_click

# -------- CONFIGURAÇÕES --------

NOME_JANELA_JOGO = "Ragnarok"  # Ajuste o título correto da janela do jogo
MINIMAP_OFFSET_X = 500       # Posição X do minimapa dentro da janela (px)
MINIMAP_OFFSET_Y = 300       # Posição Y do minimapa dentro da janela (px)
TILE_SIZE_PX = 16            # Tamanho do tile em pixels no minimapa

PORTA_SERIAL_ARDUINO = 'COM5'  # Porta serial onde o Arduino está conectado
BAUDRATE = 9600

# Endereços memória (confirme para seu jogo)
MINIMAP_X_ADDR = 0x0148A444
MINIMAP_Y_ADDR = 0x0148A448

# ------------------------------

coordenadas = []

# --- Função para ler o .gat e montar matriz ---
def ler_gat_para_matriz(caminho):
    with open(caminho, 'rb') as f:
        if f.read(6) != b'GRAT\x01\x02':
            raise ValueError("Arquivo .gat inválido")
        largura = struct.unpack('<I', f.read(4))[0]
        altura = struct.unpack('<I', f.read(4))[0]

        matriz = np.zeros((altura, largura), dtype=np.uint8)
        for y in range(altura):
            for x in range(largura):
                f.read(16)  # ignora alturas
                tipo = struct.unpack('<B', f.read(1))[0]
                f.read(3)   # ignora bytes restantes
                matriz[y, x] = tipo
        return matriz

# --- Interface para marcar pontos ---
def marcar_pontos_visualmente(caminho_gat):
    matriz = ler_gat_para_matriz(caminho_gat)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(matriz, cmap='gray', interpolation='nearest')
    ax.invert_yaxis()
    ax.set_title("Clique para marcar pontos (feche para continuar)")
    scatter = ax.scatter([], [], c='red', s=10)

    def onclick(event):
        if event.xdata and event.ydata:
            x, y = int(event.xdata), int(event.ydata)
            coordenadas.append((x, y))
            print(f"Marcado: ({x}, {y})")
            xs, ys = zip(*coordenadas)
            scatter.set_offsets(np.c_[xs, ys])
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    return matriz

# --- Busca PID do processo Ragexe ---
def encontrar_pid_ragexe():
    for proc in psutil.process_iter(['pid', 'name']):
        if 'ragexe' in proc.info['name'].lower():
            return proc.info['pid']
    raise Exception("Processo Ragexe não encontrado.")

# --- Lê a posição atual do personagem no minimapa via memória ---
def ler_posicao_personagem(pm):
    try:
        x = pm.read_int(MINIMAP_X_ADDR)
        y = pm.read_int(MINIMAP_Y_ADDR)
        return x, y
    except Exception as e:
        raise Exception("Erro ao ler coordenadas: " + str(e))

# --- Pega janela do jogo e retorna (left, top, width, height) ---
def pegar_posicao_janela(nome_janela):
    janelas = gw.getWindowsWithTitle(nome_janela)
    if not janelas:
        raise Exception(f"Janela '{nome_janela}' não encontrada")
    janela = janelas[0]
    return janela.left, janela.top, janela.width, janela.height

# --- Converte tile (coordenada do jogo) para pixel na tela para clique ---
def tile_para_pixel(tile_x, tile_y, minimap_screen_x, minimap_screen_y, tile_size_px=TILE_SIZE_PX):
    pixel_x = minimap_screen_x + tile_x * tile_size_px
    pixel_y = minimap_screen_y + tile_y * tile_size_px
    return pixel_x, pixel_y

# --- Envia comando para Arduino fazer clique ---
def clicar_com_arduino(arduino):
    arduino.write(b'C')

# --- Heurística do A* (Manhattan) ---
def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# --- Implementação do A* ---
def a_star(matriz, start, goal):
    largura = matriz.shape[1]
    altura = matriz.shape[0]

    if matriz[goal[1], goal[0]] != 0:
        print("Destino não é walkable:", goal)
        return []

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristica(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        x, y = current
        vizinhos = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        for nx, ny in vizinhos:
            if 0 <= nx < largura and 0 <= ny < altura:
                if matriz[ny, nx] == 0:
                    tentative_g = g_score[current] + 1
                    vizinho = (nx, ny)
                    if tentative_g < g_score.get(vizinho, float('inf')):
                        came_from[vizinho] = current
                        g_score[vizinho] = tentative_g
                        f_score[vizinho] = tentative_g + heuristica(vizinho, goal)
                        if vizinho not in [i[1] for i in open_set]:
                            heapq.heappush(open_set, (f_score[vizinho], vizinho))
    print("Caminho não encontrado entre", start, "e", goal)
    return []

# --- Função principal que navega pelos pontos e manda clique pelo Arduino ---
def navegar_pontos(pm, matriz, pontos, arduino, delay=1.2):
    pos_janela = pegar_posicao_janela(NOME_JANELA_JOGO)
    for idx, destino in enumerate(pontos):
        pos_atual = ler_posicao_personagem(pm)
        print(f"[{idx+1}/{len(pontos)}] Posição personagem: {pos_atual}, destino: {destino}")

        caminho = a_star(matriz, pos_atual, destino)
        if not caminho:
            print("Ignorando destino inválido.")
            continue

        for tile in caminho:
            # Calcula posição do clique no minimapa para mover mouse físico com Arduino
            px, py = tile_para_pixel(tile[0], tile[1], pos_janela[0] + MINIMAP_OFFSET_X, pos_janela[1] + MINIMAP_OFFSET_Y)

            print(f"Movendo mouse para pixel ({px}, {py}) - CLIQUE VIA ARDUINO")

            # Aqui a lógica do Arduino é enviar movimentos relativos,
            # mas como o Arduino move o mouse em pequenos passos,
            # para simplificar você pode posicionar o mouse manualmente
            # ou implementar comando para mover até o pixel.

            # Por simplicidade, só enviamos clique, assume que o cursor está no lugar certo.
            clicar_com_arduino(arduino)

            time.sleep(delay)

if __name__ == '__main__':
    caminho_gat = 'gat_files/prt_fild08.gat'

    print("Marque os pontos no mapa, fechando a janela de visualização para continuar...")
    matriz = marcar_pontos_visualmente(caminho_gat)

    print("Abrindo processo do jogo...")
    pid = encontrar_pid_ragexe()
    pm = pymem.Pymem(pid)

    print(f"Conectando ao Arduino na porta {PORTA_SERIAL_ARDUINO}...")
    arduino = serial.Serial(PORTA_SERIAL_ARDUINO, BAUDRATE, timeout=1)
    time.sleep(2)  # Espera Arduino resetar

    print("Pressione F5 para começar a navegação pelos pontos.")
    print("Pressione ESC para sair.")
    try:
        while True:
            if keyboard.is_pressed('F5'):
                print("\nIniciando navegação...")
                navegar_pontos(pm, matriz, coordenadas, arduino)
                time.sleep(0.1)

            if keyboard.is_pressed('esc'):
                print("\nSaindo...")
                break

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")

    arduino.close()
