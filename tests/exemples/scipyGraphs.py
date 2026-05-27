import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

def plot_ambiente(obstacles, nodes, path_coords=None, matriz_adj=None):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("SciPy: Grafo de Visibilidade Completo e Dijkstra")
    
    # 1. Desenha TODAS as conexões possíveis (O Grafo)
    if matriz_adj is not None:
        n_nos = len(nodes)
        for i in range(n_nos):
            for j in range(i + 1, n_nos):
                # Se a matriz de adjacência tem um valor > 0, existe uma linha de visão limpa
                if matriz_adj[i, j] > 0:
                    pt1, pt2 = nodes[i], nodes[j]
                    ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], color='gray', alpha=0.3, linewidth=1)
        
        # Cria uma linha "fantasma" só para a legenda ficar bonita
        ax.plot([], [], color='gray', alpha=0.5, linewidth=1, label='Conexões Possíveis')
        
    # 2. Desenha os obstáculos
    for obs in obstacles:
        x, y = obs.exterior.xy
        ax.fill(x, y, alpha=0.5, color='red', label='Obstáculo')
        
    # 3. Desenha os nós (pontos)
    ax.scatter(nodes[:, 0], nodes[:, 1], c='blue', s=30, label='Nós (Vértices)', zorder=5)
    ax.scatter(nodes[0, 0], nodes[0, 1], c='green', s=100, label='Início', marker='o', zorder=6)
    ax.scatter(nodes[1, 0], nodes[1, 1], c='purple', s=100, label='Alvo', marker='X', zorder=6)

    # 4. Desenha o caminho final encontrado (em destaque)
    if path_coords is not None and len(path_coords) > 0:
        ax.plot(path_coords[:, 0], path_coords[:, 1], c='green', linewidth=3, label='Caminho Mais Curto', zorder=4)

    # Limpa labels duplicadas na legenda
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left')
    ax.grid(True, linestyle='--')
    plt.show()

# 1. Definir Cenário
inicio = (1.0, 1.0)
alvo = (9.0, 8.0)
obs1 = Polygon([(3, 2), (5, 2), (5, 6), (3, 6)])
obs2 = Polygon([(6, 4), (8, 4), (8, 9), (6, 9)])
obstaculos = [obs1, obs2]

# 2. Juntar todos os nós (Início = Índice 0, Alvo = Índice 1)
nos = [inicio, alvo]
for obs in obstaculos:
    # list()[:-1] para não duplicar o último ponto que fecha o polígono
    nos.extend(list(obs.exterior.coords)[:-1]) 
nos = np.array(nos)
n_nos = len(nos)

# 3. Construir a Matriz de Adjacência
matriz_adj = np.zeros((n_nos, n_nos))

for i in range(n_nos):
    for j in range(i + 1, n_nos):
        linha = LineString([nos[i], nos[j]])
        
        # DICA DE OURO: Usamos um buffer negativo minúsculo no obstáculo.
        # Isso impede que o Shapely ache que a linha bateu no obstáculo só porque ela raspou na quina.
        colidiu = any(linha.intersects(obs.buffer(-1e-5)) for obs in obstaculos)
        
        if not colidiu:
            dist = np.linalg.norm(nos[i] - nos[j])
            matriz_adj[i, j] = dist
            matriz_adj[j, i] = dist # Grafo bidirecional

# 4. Solucionar com SciPy
grafo_esparso = csr_matrix(matriz_adj)
distancias, predecessores = dijkstra(grafo_esparso, directed=False, indices=0, return_predecessors=True)

# 5. Reconstruir Caminho
caminho_indices = []
atual = 1 # Queremos chegar no alvo (índice 1)

while atual != -9999:
    caminho_indices.append(atual)
    if atual == 0:
        break
    atual = predecessores[atual]

caminho_indices.reverse()
coordenadas_caminho = nos[caminho_indices]

# Chamando a função passando a matriz_adj agora
plot_ambiente(obstaculos, nos, coordenadas_caminho, matriz_adj)