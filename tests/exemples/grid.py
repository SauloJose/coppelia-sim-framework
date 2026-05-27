import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Polygon, box, Point

# ==========================================
# 1. DEFINIÇÃO DO MAPA E OBSTÁCULOS
# ==========================================

# Lista de obstáculos (Polígonos, Retângulos e Círculos)
obstaculos = [
    Polygon([(2.5, 3.5), (6.5, 2.0), (7.0, 6.0), (3.0, 7.0)]),  # Obstáculo original
    box(1.0, 1.0, 2.0, 5.0),                                    # Retângulo (parede)
    Point(8.0, 8.0).buffer(1.2),                                # Círculo (aproximado por polígono via buffer)
    Polygon([(7.0, 1.0), (9.0, 1.0), (8.0, 3.0)])               # Triângulo
]

# Limites do mapa
x_min, x_max = 0.0, 10.0
y_min, y_max = 0.0, 10.0

# Resolução da célula (ex: 1 metro por 1 metro)
tamanho_celula = 0.1

# Calcular quantas células cabem no mapa
linhas = int((y_max - y_min) / tamanho_celula)
colunas = int((x_max - x_min) / tamanho_celula)

# ==========================================
# 2. DISCRETIZAÇÃO DO GRID
# ==========================================

# Matriz do Grid (0 = Livre, 1 = Ocupado)
grid_ocupacao = np.zeros((linhas, colunas))

# Discretizar o mapa checando interseção
for i in range(linhas):
    for j in range(colunas):
        # Acha as coordenadas reais (X, Y) do canto da célula atual
        cx_min = x_min + (j * tamanho_celula)
        cy_min = y_min + (i * tamanho_celula)
        cx_max = cx_min + tamanho_celula
        cy_max = cy_min + tamanho_celula
        
        # Cria um retângulo Shapely representando a área dessa célula exata
        poligono_celula = box(cx_min, cy_min, cx_max, cy_max)
        
        # Verifica se essa célula encosta ou contém QUALQUER UM dos obstáculos
        if any(poligono_celula.intersects(obs) for obs in obstaculos):
            grid_ocupacao[i, j] = 1

# ==========================================
# 3. VISUALIZAÇÃO
# ==========================================

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_title("Mapeamento em Grid de Ocupação com Múltiplos Objetos")

# Desenha o Grid de cores usando o Matplotlib (Células ocupadas em preto)
ax.imshow(grid_ocupacao, origin='lower', extent=[x_min, x_max, y_min, y_max], cmap='Greys', alpha=0.6)

# Plota o contorno de todos os obstáculos reais por cima para compararmos
for idx, obs in enumerate(obstaculos):
    x, y = obs.exterior.xy
    # Coloca a label apenas no primeiro para não duplicar na legenda
    ax.plot(x, y, color='red', linewidth=2, label='Obstáculos Reais' if idx == 0 else "")

# Configura as linhas do grid para ficarem visíveis (a cada "tamanho_celula" metros)
ax.set_xticks(np.arange(x_min, x_max + 1, tamanho_celula))
ax.set_yticks(np.arange(y_min, y_max + 1, tamanho_celula))
ax.grid(color='cyan', linestyle='-', linewidth=0.5)

ax.legend(loc='upper left')
plt.show()