import struct
import numpy as np
import matplotlib.pyplot as plt
import psutil
import pymem
import time
import heapq
import keyboard
import pygetwindow as gw
import ctypes
from controllers.mouse_controller import move_mouse_and_click, get_mouse_position, only_click

# Configurações (ajuste conforme seu jogo e setup)
NOME_JANELA_JOGO = "Ragnarok"
TILE_SIZE_PX = 16      # Tamanho do tile no minimapa em pixels

# Endereços de memória do jogo
MINIMAP_X_ADDR = 0x0148A444
MINIMAP_Y_ADDR = 0x0148A448

coordenadas = []

# Windows API para pegar posição do mouse
user32 = ctypes.windll.user32

def get_mouse_pos_windows():
    class POINT(ctypes.Structure):
        _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]
    pt = POINT()
    user32.GetCursorPos(ctypes.byref(pt))
    return pt.x, pt.y

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

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

def encontrar_pid_ragexe():
    for proc in psutil.process_iter(['pid', 'name']):
        if 'ragexe' in proc.info['name'].lower():
            return proc.info['pid']
    raise Exception("Processo Ragexe não encontrado.")

def ler_posicao_personagem(pm):
    try:
        x = pm.read_int(MINIMAP_X_ADDR)
        y = pm.read_int(MINIMAP_Y_ADDR)
        return x, y
    except Exception as e:
        raise Exception("Erro ao ler coordenadas: " + str(e))

def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
    
import math


class Coordinate():
  def __init__(self, x, y, _type="game"):
    self.x = x
    self.y = y
    self.type = _type

  def __str__(self):
    return f"({self.x}, {self.y}, {self.type})"

  def distance_to(self, target):
    dx = self.x - target.x
    dy = self.y - target.y

    return math.sqrt(dx**2 + dy**2)

  def to_screen(self, game):
    if self.type == "screen":
      return
    else:
      pos = game.window.translate_to_screen_coords(game.world.player.coordinates(), self)
      self.x = pos.x
      self.y = pos.y
      self.type = "game"

  def to_game(self):
    pass


def navegar_pontos(pm, matriz, pontos, janela_titulo, delay=0.6):
    janelas = gw.getWindowsWithTitle(janela_titulo)
    if not janelas:
        print(f"Janela '{janela_titulo}' não encontrada.")
        return
    janela = janelas[0]

    for idx, destino in enumerate(pontos):
        # Posição atual do personagem no jogo como Coordinate
        pos_atual_tuple = ler_posicao_personagem(pm)
        pos_atual = Coordinate(pos_atual_tuple[0], pos_atual_tuple[1], _type="game")
        destino_coord = Coordinate(destino[0], destino[1], _type="game")

        print(f"[{idx+1}/{len(pontos)}] Posição personagem: {pos_atual}, destino: {destino_coord}")

        caminho = a_star(matriz, (pos_atual.x, pos_atual.y), (destino_coord.x, destino_coord.y))
        if not caminho:
            print("Ignorando destino inválido.")
            continue

        for tile in caminho:
            # Atualiza posição do personagem
            pos_atual_tuple = ler_posicao_personagem(pm)
            pos_atual = Coordinate(pos_atual_tuple[0], pos_atual_tuple[1], _type="game")
            tile_coord = Coordinate(tile[0], tile[1], _type="game")

            # Calcula o delta do movimento em tiles
            dx_tile = tile_coord.x - pos_atual.x
            dy_tile = tile_coord.y - pos_atual.y

            # Converte delta tiles para pixels, mantendo eixo Y invertido
            dx_px = dx_tile * TILE_SIZE_PX
            dy_px = -dy_tile * TILE_SIZE_PX

            # Pega posição atual do mouse na tela (screen)
            x_mouse, y_mouse = get_mouse_pos_windows()
            mouse_coord = Coordinate(x_mouse, y_mouse, _type="screen")

            # Calcula nova posição do mouse na tela, somando o delta em px
            x_novo = x_mouse + dx_px
            y_novo = y_mouse + dy_px

            # Limita para dentro da janela
            x_novo_limitado = clamp(x_novo, janela.left, janela.left + janela.width - 1)
            y_novo_limitado = clamp(y_novo, janela.top, janela.top + janela.height - 1)

            dx_real = x_novo_limitado - x_mouse
            dy_real = y_novo_limitado - y_mouse

            if dx_real == 0 and dy_real == 0:
                print(f"Passo ignorado: movimento resultaria fora da janela do jogo.")
                continue

            print(f"Movendo mouse ajustado: dx={dx_real}px, dy={dy_real}px e clicando via Arduino.")
            move_mouse_and_click(dx_real, dy_real, janela_titulo)

            time.sleep(delay)


if __name__ == '__main__':
    caminho_gat = 'gat_files/prt_fild08.gat'

    print("Marque os pontos no mapa (feche a janela para continuar)...")
    matriz = marcar_pontos_visualmente(caminho_gat)

    print("Conectando ao processo do jogo...")
    pid = encontrar_pid_ragexe()
    pm = pymem.Pymem(pid)

    print("Pressione F5 para iniciar a navegação, ESC para sair.")
    try:
        while True:
            if keyboard.is_pressed('F5'):
                navegar_pontos(pm, matriz, coordenadas, NOME_JANELA_JOGO)
                time.sleep(0.1)

            if keyboard.is_pressed('esc'):
                print("Saindo...")
                break

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrompido pelo usuário.")
