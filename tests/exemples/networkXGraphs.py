import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import Polygon, LineString

# Cenário expandido com 10 obstáculos
inicio = (1.0, 1.0)
alvo = (9.0, 8.0)

# Definição dos 10 polígonos (misturando retângulos e triângulos)
obs1 = Polygon([(3.0, 2.0), (5.0, 2.0), (5.0, 4.0), (3.0, 4.0)])
obs2 = Polygon([(6.0, 4.0), (8.0, 4.0), (8.0, 6.0), (6.0, 6.0)])
obs3 = Polygon([(1.0, 5.0), (2.5, 5.0), (2.5, 7.0), (1.0, 7.0)])
obs4 = Polygon([(4.0, 7.0), (5.5, 7.0), (5.5, 9.0), (4.0, 9.0)])
obs5 = Polygon([(7.0, 1.0), (9.0, 1.0), (8.0, 3.0)])
obs6 = Polygon([(0.5, 3.0), (2.0, 3.0), (2.0, 4.0), (0.5, 4.0)])
obs7 = Polygon([(3.5, 0.5), (4.5, 0.5), (4.5, 1.5), (3.5, 1.5)])
obs8 = Polygon([(5.5, 2.5), (6.5, 2.5), (6.5, 3.5), (5.5, 3.5)])
obs9 = Polygon([(8.5, 4.5), (9.5, 4.5), (9.5, 6.5), (8.5, 6.5)])
obs10 = Polygon([(1.5, 8.5), (3.5, 8.5), (2.5, 9.5)])

obstaculos = [obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10]

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