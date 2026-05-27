import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import Polygon, LineString

# Mesmo cenário do exemplo anterior
inicio = (1.0, 1.0)
alvo = (9.0, 8.0)
obs1 = Polygon([(3, 2), (5, 2), (5, 6), (3, 6)])
obs2 = Polygon([(6, 4), (8, 4), (8, 9), (6, 9)])
obstaculos = [obs1, obs2]

nos = [inicio, alvo]
for obs in obstaculos:
    nos.extend(list(obs.exterior.coords)[:-1])
nos = np.array(nos)
n_nos = len(nos)

# 1. Inicializar o Grafo do NetworkX
G = nx.Graph()

# 2. Adicionar os nós (usando os índices)
for i in range(n_nos):
    G.add_node(i, pos=nos[i])

# 3. Adicionar as arestas se houver linha de visão
for i in range(n_nos):
    for j in range(i + 1, n_nos):
        linha = LineString([nos[i], nos[j]])
        if not any(linha.intersects(obs.buffer(-1e-5)) for obs in obstaculos):
            dist = np.linalg.norm(nos[i] - nos[j])
            G.add_edge(i, j, weight=dist) # O peso é a distância física

# 4. Solucionar com A*
def heuristica_euclidiana(u, v):
    return np.linalg.norm(nos[u] - nos[v])

# nx.astar_path retorna diretamente a lista dos índices do caminho
caminho_indices = nx.astar_path(G, source=0, target=1, heuristic=heuristica_euclidiana, weight='weight')
coordenadas_caminho = nos[caminho_indices]

# 5. Plotando (Podemos usar os nós e arestas do próprio NetworkX para visualizar a "malha")
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_title("NetworkX: Grafo Visível e Busca A*")

for obs in obstaculos:
    x, y = obs.exterior.xy
    ax.fill(x, y, alpha=0.5, color='red')

# Desenha todas as linhas de visão válidas geradas pelo NetworkX em cinza claro
for u, v in G.edges():
    pt1, pt2 = nos[u], nos[v]
    ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], color='gray', alpha=0.3, linewidth=1)

# Desenha o caminho vencedor
ax.plot(coordenadas_caminho[:, 0], coordenadas_caminho[:, 1], c='blue', linewidth=3, label='Caminho A*')
ax.scatter(nos[0, 0], nos[0, 1], c='green', s=100, label='Início', zorder=5)
ax.scatter(nos[1, 0], nos[1, 1], c='purple', s=100, label='Alvo', marker='X', zorder=5)

ax.legend()
plt.show()