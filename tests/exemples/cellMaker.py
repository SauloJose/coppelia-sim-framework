"""
@title: trapezoidal_decomposition_astar.py
@author: Terrance Williams (Modificado)
@description: 
    Realiza a decomposição trapezoidal de um mapa, reconstrói as células do espaço 
    livre, gera um grafo de visibilidade global conectando todos os nós (centroides, 
    start e goal) que possuem linha de visão direta livre de obstáculos, e encontra 
    o menor caminho usando o algoritmo A*.
"""

from __future__ import annotations
import heapq
from typing import List, Dict, Tuple, Set, Any
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpath
from shapes import Edge, Polygon, Triangle, Pentagon, point, arr_eq


# =============================================================================
# MÓDULO 1: FUNÇÕES AUXILIARES E GEOMETRIA
# =============================================================================

def get_plot_data(edges: list) -> tuple:
    """Converte segmentos de retas (edges) para o formato compatível com o plot."""
    x = [(line.pt_a[0], line.pt_b[0]) for line in edges]
    y = [(line.pt_a[1], line.pt_b[1]) for line in edges]
    return x, y


def calcular_centroide(vertices: list) -> tuple:
    """
    Calcula o centroide geométrico exato de um polígono plano 2D 
    utilizando a fórmula baseada no Teorema de Green.
    """
    pts = vertices + [vertices[0]]
    area = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(len(vertices)):
        x0, y0 = pts[i]
        x1, y1 = pts[i+1]
        factor = (x0 * y1 - x1 * y0)
        area += factor
        cx += (x0 + x1) * factor
        cy += (y0 + y1) * factor
    area *= 0.5
    
    if abs(area) < 1e-5:  # Evita divisão por zero em células degeneradas
        return np.mean([p[0] for p in vertices]), np.mean([p[1] for p in vertices])
        
    cx /= (6.0 * area)
    cy /= (6.0 * area)
    return cx, cy


# =============================================================================
# MÓDULO 2: RECONSTRUÇÃO DAS CÉLULAS DO ESPAÇO LIVRE
# =============================================================================

def reconstruir_celulas_livres(obstacles: list, min_x: float, max_x: float, 
                               min_y: float, max_y: float) -> List[Dict[str, Any]]:
    """
    Varre o espaço verticalmente mapeando e criando os polígonos (trapézios/triângulos)
    que representam as células exclusivas do espaço livre.
    """
    # 1. Coleta todas as coordenadas X únicas (eventos de colisão/vértices)
    all_xs = set([min_x, max_x])
    for obst in obstacles:
        for v in obst.vertices:
            all_xs.add(v[0])
    sorted_xs = sorted(list(all_xs))

    # 2. Extrai as arestas dos obstáculos e as bordas externas do mapa
    raw_edges = []
    for obst in obstacles:
        for edge in obst.edges:
            raw_edges.append(((edge.pt_a[0], edge.pt_a[1]), (edge.pt_b[0], edge.pt_b[1])))
    raw_edges.append(((min_x, min_y), (max_x, min_y)))  # Borda inferior
    raw_edges.append(((min_x, max_y), (max_x, max_y)))  # Borda superior

    # Matplotlib Paths para verificar se uma célula candidata está colidindo com obstáculo
    obst_paths = [mpath.Path([(v[0], v[1]) for v in obst.vertices]) for obst in obstacles]

    celulas_livres = []
    id_celula = 0

    # 3. Varredura por intervalos de X
    for idx in range(len(sorted_xs) - 1):
        x_left = sorted_xs[idx]
        x_right = sorted_xs[idx+1]
        if abs(x_left - x_right) < 1e-5:
            continue
        
        x_mid = (x_left + x_right) / 2.0
        
        # Encontra arestas que cruzam a linha central do intervalo X atual
        intersecting_edges = []
        for edge in raw_edges:
            (x1, y1), (x2, y2) = edge
            if min(x1, x2) <= x_mid <= max(x1, x2) and abs(x1 - x2) > 1e-5:
                y_mid = y1 + (y2 - y1) * (x_mid - x1) / (x2 - x1)
                intersecting_edges.append((y_mid, edge))
        
        # Ordena verticalmente (de baixo para cima)
        intersecting_edges.sort(key=lambda item: item[0])
        
        # Valida os sub-intervalos verticais gerados pelas interseções
        for j in range(len(intersecting_edges) - 1):
            y_mid_low, edge_low = intersecting_edges[j]
            y_mid_high, edge_high = intersecting_edges[j+1]
            
            # Ponto central para checagem de colisão interna
            y_test = (y_mid_low + y_mid_high) / 2.0
            pt_test = (x_mid, y_test)
            
            # Se livre de obstáculos, calcula as quinas exatas da célula
            if not any(path.contains_point(pt_test) for path in obst_paths):
                (xl1, yl1), (xl2, yl2) = edge_low
                y_low_left = yl1 + (yl2 - yl1) * (x_left - xl1) / (xl2 - xl1)
                y_low_right = yl1 + (yl2 - yl1) * (x_right - xl1) / (xl2 - xl1)
                
                (xh1, yh1), (xh2, yh2) = edge_high
                y_high_left = yh1 + (yh2 - yh1) * (x_left - xh1) / (xh2 - xh1)
                y_high_right = yh1 + (yh2 - yh1) * (x_right - xh1) / (xh2 - xh1)
                
                trap_vertices = [
                    (x_left, y_low_left),
                    (x_right, y_low_right),
                    (x_right, y_high_right),
                    (x_left, y_high_left)
                ]
                
                cx, cy = calcular_centroide(trap_vertices)
                
                celulas_livres.append({
                    'id': id_celula,
                    'vertices': trap_vertices,
                    'centroid': (cx, cy),
                    'x_left': x_left,
                    'x_right': x_right,
                    'y_bounds_left': (y_low_left, y_high_left),
                    'y_bounds_right': (y_low_right, y_high_right)
                })
                id_celula += 1
                
    return celulas_livres


# =============================================================================
# MÓDULO 3: CONSTRUÇÃO DO GRAFO DE ADJACÊNCIA (COM CHECAGEM DE COLISÃO)
# =============================================================================

def construir_grafo(cells: List[Dict[str, Any]], inicio: tuple, alvo: tuple, obstacles: list) -> Tuple[dict, dict]:
    """
    Conecta TODOS os nós do sistema (centroides, start e goal) entre si.
    Arestas que colidirem ou cruzarem com qualquer obstáculo são ignoradas.
    """
    # Mapeia IDs para suas respectivas coordenadas geográficas 2D
    posicoes = {c['id']: c['centroid'] for c in cells}
    posicoes['start'] = inicio
    posicoes['goal'] = alvo
    
    nos = list(posicoes.keys())
    grafo = {no: [] for no in nos}

    def intersect_estrito(A: tuple, B: tuple, C: tuple, D: tuple) -> bool:
        """Verifica se o segmento de reta AB cruza transversalmente o segmento CD."""
        def cross_product(p1, p2, p3):
            return (p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])
        
        cp1, cp2 = cross_product(A, B, C), cross_product(A, B, D)
        cp3, cp4 = cross_product(C, D, A), cross_product(C, D, B)
        
        # Se os sinais forem estritamente opostos, há cruzamento direto de segmentos
        if ((cp1 > 1e-5 and cp2 < -1e-5) or (cp1 < -1e-5 and cp2 > 1e-5)) and \
           ((cp3 > 1e-5 and cp4 < -1e-5) or (cp3 < -1e-5 and cp4 > 1e-5)):
            return True
        return False

    def linha_colide_com_obstaculos(p1: tuple, p2: tuple) -> bool:
        """Mapeia se o raio de ligação entre dois nós toca ou atravessa algum obstáculo."""
        # Teste 1: Interseção direta de segmentos com as bordas dos obstáculos
        for obst in obstacles:
            for edge in obst.edges:
                A = (edge.pt_a[0], edge.pt_a[1])
                B = (edge.pt_b[0], edge.pt_b[1])
                if intersect_estrito(p1, p2, A, B):
                    return True  # Colisão por cruzamento de parede
        
        # Teste 2: Amostragem interna (garante que a linha não passe totalmente por dentro)
        for t in [0.25, 0.5, 0.75]:
            cx = p1[0] + t * (p2[0] - p1[0])
            cy = p1[1] + t * (p2[1] - p1[1])
            pt_teste = point(cx, cy)
            for obst in obstacles:
                if obst.contains(pt_teste):
                    return True  # Colisão por confinamento interno
                    
        return False

    # Varre combinatoriamente todos os pares possíveis (All-to-All)
    for i in range(len(nos)):
        for j in range(i + 1, len(nos)):
            no1 = nos[i]
            no2 = nos[j]
            p1 = posicoes[no1]
            p2 = posicoes[no2]
            
            # Se a linha de visão for limpa, a conexão bilateral é adicionada ao grafo
            if not linha_colide_com_obstaculos(p1, p2):
                grafo[no1].append(no2)
                grafo[no2].append(no1)

    return grafo, posicoes


# =============================================================================
# MÓDULO 4: ALGORITMO DE BUSCA A* (A-STAR)
# =============================================================================

def buscar_caminho_astar(grafo: dict, posicoes: dict, start: Any, goal: Any) -> List[Any]:
    """
    Executa a busca de menor caminho utilizando o algoritmo clássico A*.
    Utiliza distância Euclidiana como custo e heurística.
    """
    def heuristica(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    fila_prioridade = [(0 + heuristica(posicoes[start], posicoes[goal]), 0, start, [start])]
    visitados = set()

    while fila_prioridade:
        f, g, atual, caminho = heapq.heappop(fila_prioridade)

        if atual == goal:
            return caminho

        if atual in visitados:
            continue
        visitados.add(atual)

        for vizinho in grafo[atual]:
            if vizinho in visitados:
                continue
            
            custo_passo = heuristica(posicoes[atual], posicoes[vizinho])
            novo_g = g + custo_passo
            novo_f = novo_g + heuristica(posicoes[vizinho], posicoes[goal])
            
            heapq.heappush(fila_prioridade, (novo_f, novo_g, vizinho, caminho + [vizinho]))

    return []


# =============================================================================
# MÓDULO 5: VISUALIZAÇÃO GRÁFICA DO CENÁRIO
# =============================================================================

def plotar_cenario(obstacles: list, vertical_segments: list, cells: list, 
                   grafo: dict, posicoes: dict, caminho: list, bounds: tuple):
    """Gera o mapa visual completo contendo células, obstáculos, grafo e rota A*."""
    min_x, max_x, min_y, max_y = bounds
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.grid(True, linestyle=':', alpha=0.6)

    # 1. Desenhar TODAS as Células Livres (Trapézios em tons suaves)
    for c in cells:
        poly_x, poly_y = zip(*c['vertices'])
        ax.fill(poly_x, poly_y, edgecolor='darkgray', facecolor='whitesmoke', 
                linestyle='-', alpha=0.8, zorder=1)

    # 2. Desenhar os Obstáculos (Polígonos Azuis)
    for shape in obstacles:
        x, y = zip(*shape.vertices)
        ax.fill(x, y, color='royalblue', alpha=0.85, edgecolor='midnightblue', zorder=4)

    # 3. Desenhar Delimitadores Verticais da Decomposição (Tracejado Preto)
    x_seg, y_seg = get_plot_data(vertical_segments)
    for j, k in zip(x_seg, y_seg):
        ax.plot(j, k, color='black', linestyle='--', linewidth=1.2, alpha=0.7, zorder=2)

    # 4. Desenhar as Arestas Validadas do Grafo (Conexões globais seguras)
    arestas_desenhadas = set()
    for no, vizinhos in grafo.items():
        for vizinho in vizinhos:
            par_aresta = tuple(sorted([str(no), str(vizinho)]))
            if par_aresta not in arestas_desenhadas:
                p1, p2 = posicoes[no], posicoes[vizinho]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', 
                        linestyle=':', linewidth=1.1, alpha=0.5, zorder=3)
                arestas_desenhadas.add(par_aresta)

    # 5. Desenhar Todos os Centroides
    cx_lista = [c['centroid'][0] for c in cells]
    cy_lista = [c['centroid'][1] for c in cells]
    ax.scatter(cx_lista, cy_lista, color='crimson', marker='o', s=25, zorder=5, label='Centroides')

    # 6. Desenhar o Menor Caminho Encontrado pelo A*
    if caminho:
        caminho_pts = [posicoes[no] for no in caminho]
        cam_x, cam_y = zip(*caminho_pts)
        ax.plot(cam_x, cam_y, color='orangered', linewidth=3.5, linestyle='-', 
                label='Rota A*', zorder=6)
        
        ax.scatter(posicoes['start'][0], posicoes['start'][1], color='lime', edgecolor='black',
                   marker='^', s=120, label='Início (Start)', zorder=7)
        ax.scatter(posicoes['goal'][0], posicoes['goal'][1], color='gold', edgecolor='black',
                   marker='*', s=150, label='Alvo (Goal)', zorder=7)

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect('equal')
    ax.set(xlabel='Coordenada X', ylabel='Coordenada Y', title='Decomposição Trapezoidal com Grafo de Visibilidade Otimizado')
    ax.legend(loc='upper right')
    plt.show()

# =============================================================================
# EXECUÇÃO PRINCIPAL: CENÁRIO DE LABIRINTO COMPLEXO
# =============================================================================

if __name__ == "__main__":
    # Dimensões do mapa global
    MIN_X, MAX_X = 0, 50
    MIN_Y, MAX_Y = MIN_X, MAX_X

    # Posições estratégicas: Início no topo esquerdo, Alvo no canto inferior direito
    INICIO_PT = (3.0, 45.0)
    ALVO_PT = (46.0, 4.0)

    def criar_parede(x: float, y: float, largura: float, altura: float) -> Polygon:
        """
        Função auxiliar para gerar paredes retangulares baseadas em uma 
        coordenada inicial (inferior esquerda), largura e altura.
        """
        return Polygon([
            point(x, y),
            point(x + largura, y),
            point(x + largura, y + altura),
            point(x, y + altura)
        ])

    # --- GERADOR DE LABIRINTO (Complexo e com passagem garantida) ---
    # Cria barreiras horizontais principais com aberturas alternadas,
    # complementadas por obstáculos verticais que forçam desvios em cada nível.
    map_obstacles = [
        # Barreiras horizontais principais (com gaps alternados)
        criar_parede(x=0, y=40, largura=44, altura=2),   # 1. Superior: gap à direita (44-46)
        criar_parede(x=6, y=30, largura=44, altura=2),   # 2. Intermediária alta: gap à esquerda (4-6)
        criar_parede(x=0, y=20, largura=44, altura=2),   # 3. Intermediária central: gap à direita (44-46)
        criar_parede(x=6, y=10, largura=44, altura=2),   # 4. Intermediária baixa: gap à esquerda (4-6)
        criar_parede(x=8, y=6, largura=36, altura=2),    # 5. Barreira inferior: gap à direita (44-50)
        criar_parede(x=10, y=9, largura=30, altura=1),   # 6. Desvio horizontal extra na região baixa

        # Obstáculos verticais no primeiro corredor (y=32..40)
        criar_parede(x=30, y=32, largura=2, altura=6),   # 7. Bloqueio parcial, forçando subida
        criar_parede(x=15, y=34, largura=2, altura=4),   # 8. Pequeno pilar
        criar_parede(x=20, y=34, largura=2, altura=2),   # 9. Bloco extra

        # Obstáculos verticais no segundo corredor (y=22..30)
        criar_parede(x=15, y=22, largura=2, altura=6),   # 10. Bloqueio parcial, com passagem superior
        criar_parede(x=35, y=24, largura=2, altura=4),   # 11. Pilar deslocado
        criar_parede(x=25, y=26, largura=2, altura=3),   # 12. Barreira baixa adicional

        # Obstáculos verticais no terceiro corredor (y=12..20)
        criar_parede(x=30, y=12, largura=2, altura=6),   # 13. Bloqueio central
        criar_parede(x=12, y=14, largura=2, altura=4),   # 14. Pilar esquerdo
        criar_parede(x=20, y=16, largura=2, altura=2),   # 15. Pequeno bloqueio

        # Obstáculo próximo ao objetivo (y=2..6), deixando o alvo livre
        criar_parede(x=44, y=2, largura=1, altura=4),    # 16. Barreira fina para último desvio
    ]

    # --- ETAPA 1: Processamento das Linhas Verticais de Fronteira (Código Base Original) ---
    vertical_segments: List[Edge] = []
    for obst in map_obstacles:
        filtered = [x for x in map_obstacles if x is not obst]
        for vertex in obst.vertices:
            
            # Criamos uma lista dinâmica para armazenar os segmentos válidos
            # O primeiro elemento da tupla indica se é TOP (0) ou BOTTOM (1) para manter a lógica original
            potential_verts = []
            
            # Só tenta criar o segmento para o teto se o vértice já não estiver lá
            if abs(vertex[1] - MAX_Y) > 1e-5:
                potential_verts.append((0, Edge(vertex, point(vertex[0], MAX_Y))))
                
            # Só tenta criar o segmento para o chão se o vértice já não estiver lá
            if abs(vertex[1] - MIN_Y) > 1e-5:
                potential_verts.append((1, Edge(vertex, point(vertex[0], MIN_Y))))
            
            # Varre apenas os segmentos verticais que fazem sentido geométrico
            for i, seg in potential_verts:
                self_intersects = False
                for edge in obst.edges:
                    intr = seg.find_intersection(edge)
                    if intr is None or arr_eq(intr, vertex):
                        continue
                    elif (i == 0) and (obst.touches(intr) and intr[1] > vertex[1]):
                        self_intersects = True
                    elif (i == 1) and (obst.touches(intr) and intr[1] < vertex[1]):
                        self_intersects = True
                else:
                    if self_intersects:
                        continue
                    else:
                        intersection_ys = []
                        intr_found = False
                        for other in filtered:
                            for edge in other.edges:
                                intr = seg.find_intersection(edge)
                                if (
                                        (intr is None) or
                                        (not edge.is_on(intr)) or
                                        arr_eq(intr, vertex) or
                                        (i == 0 and intr[1] < vertex[1]) or
                                        (i == 1 and intr[1] > vertex[1]) or
                                        other.contains(intr)
                                ):
                                    continue
                                else:
                                    intersection_ys.append(intr[1])
                                    intr_found = True
                        else:
                            if not intr_found:
                                vertical_segments.append(seg)
                            else:
                                intr_y = min(intersection_ys) if i == 0 else max(intersection_ys)
                                vertical_segments.append(Edge(vertex, point(vertex[0], intr_y)))
    # --- ETAPA 2: Reconstrução das Polígonos de Células ---
    celulas_livres = reconstruir_celulas_livres(map_obstacles, MIN_X, MAX_X, MIN_Y, MAX_Y)
    print(f"Total de Células livres identificadas: {len(celulas_livres)}")

    # --- ETAPA 3: Geração do Grafo de Conectividade ---
    # Nota: Usa a função modificada anteriormente que checa colisões All-to-All
    grafo, posicoes_nos = construir_grafo(celulas_livres, INICIO_PT, ALVO_PT, map_obstacles)

    # --- ETAPA 4: Navegação com o Algoritmo A* ---
    caminho_resolvido = buscar_caminho_astar(grafo, posicoes_nos, 'start', 'goal')
    print(f"Sequência de nós visitados pelo A*: {caminho_resolvido}")

    # --- ETAPA 5: Renderização do Gráfico ---
    plotar_cenario(
        obstacles=map_obstacles,
        vertical_segments=vertical_segments,
        cells=celulas_livres,
        grafo=grafo,
        posicoes=posicoes_nos,
        caminho=caminho_resolvido,
        bounds=(MIN_X, MAX_X, MIN_Y, MAX_Y)
    )