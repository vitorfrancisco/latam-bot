import struct
import numpy as np
import matplotlib.pyplot as plt

# Lista para armazenar os pontos clicados
coordenadas = []

def ler_gat_para_matriz(caminho):
    with open(caminho, 'rb') as f:
        magic = f.read(6)
        if magic != b'GRAT\x01\x02':
            raise ValueError("Arquivo .gat inválido ou não suportado")

        largura = struct.unpack('<I', f.read(4))[0]
        altura = struct.unpack('<I', f.read(4))[0]

        matriz = np.zeros((altura, largura), dtype=np.uint8)

        for y in range(altura):
            for x in range(largura):
                f.read(16)
                tipo = struct.unpack('<B', f.read(1))[0]
                f.read(3) 
                matriz[y, x] = tipo

        return matriz

def marcar_pontos_visualmente(caminho_gat, saida_img='saida_marcada.png', saida_txt='coordenadas.txt'):
    matriz = ler_gat_para_matriz(caminho_gat)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(matriz, cmap='gray', interpolation='nearest')
    ax.invert_yaxis()
    ax.set_title("Clique para marcar pontos (feche a janela para salvar)")

    scatter = ax.scatter([], [], c='red', s=10)

    def onclick(event):
        if event.xdata is not None and event.ydata is not None:
            x, y = int(event.xdata), int(event.ydata)
            coordenadas.append((x, y))
            print(f"Coordenada clicada: ({x}, {y})")

            xs, ys = zip(*coordenadas)
            scatter.set_offsets(np.c_[xs, ys])
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    fig.savefig(saida_img, dpi=300)
    print(f"✅ Imagem salva como: {saida_img}")

    # Salva coordenadas clicadas
    with open(saida_txt, 'w') as f:
        for x, y in coordenadas:
            f.write(f"{x},{y}\n")
    print(f"✅ Coordenadas salvas em: {saida_txt}")

# === USO ===
marcar_pontos_visualmente("gat_files/prt_fild08.gat")
