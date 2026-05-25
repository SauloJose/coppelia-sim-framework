"""
@title: trapezoidal_decomposition.py
@author: Terrance Williams (Adapted)
@date: 22 November 2023
@description: 
    This document uses definitions from `shapes.py` to perform trapezoidal decomposition on the representation of a map 
    of obstacles, extract cells, define left-midpoints, and find a path.
"""
from __future__ import annotations
from typing import List
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

# Importações do Shapely para lidar com os Polígonos formados pelos segmentos
from shapely.geometry import Point as ShPoint, LineString, Polygon as ShPolygon
from shapely.ops import polygonize, unary_union

from shapes import (
    Edge, Polygon, Triangle, Pentagon, point, arr_eq
)

def get_plot_data(edges: list) -> tuple:
    """Convert points to plot-compatible format"""
    x = [(line.pt_a[0], line.pt_b[0]) for line in edges]
    y = [(line.pt_a[1], line.pt_b[1]) for line in edges]
    return x, y

def flatten_data(data):
    flat = [m for item in data for m in item]
    return flat

if __name__ == "__main__":
    # Define Border Bounds; Assume Border is a Square.
    MIN_X, MAX_X = 0, 50
    MIN_Y, MAX_Y = MIN_X, MAX_X

    obstacles = [
        Triangle([
            point(36, 26),
            point(48, 37),
            point(48, 21)
        ]),
        Pentagon([
            point(16, 14),
            point(23, 22),
            point(34, 21),
            point(33, 7),
            point(24, 4)
        ]),
        Polygon([
            point(8, 17),
            point(3, 31),
            point(17, 37),
            point(20, 30),
            point(10, 27),
            point(12, 20)
        ]),
    ]
    vertical_segments: List[Edge] = []  # Cell Delimiters

    # Find valid vertical delimeters for each vertex in each obstacle.
    for obst in obstacles:
        filtered = [x for x in obstacles if x is not obst]
        for vertex in obst.vertices:
            top_vert = Edge(vertex, point(vertex[0], MAX_Y))
            bottom_vert = Edge(vertex, point(vertex[0], MIN_Y))
            verts = (top_vert, bottom_vert)
            
            for i in range(len(verts)):
                seg = verts[i]
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
                                if i == 0:
                                    intr_y = min(intersection_ys)
                                else:
                                    intr_y = max(intersection_ys)
                                vertical_segments.append(Edge(vertex, point(vertex[0], intr_y)))

    # ==========================================
    # 1. CONSTRUÇÃO E FILTRAGEM DOS TRAPÉZIOS
    # ==========================================
    lines_for_polygonize = []
    
    # Adiciona Bordas do Mapa
    lines_for_polygonize.extend([
        LineString([(MIN_X, MIN_Y), (MAX_X, MIN_Y)]),
        LineString([(MAX_X, MIN_Y), (MAX_X, MAX_Y)]),
        LineString([(MAX_X, MAX_Y), (MIN_X, MAX_Y)]),
        LineString([(MIN_X, MAX_Y), (MIN_X, MIN_Y)])
    ])
    
    # Adiciona Arestas dos Obstáculos e cria versão Shapely deles para checagem
    shapely_obstacles = []
    for obst in obstacles:
        obs_coords = [tuple(v) for v in obst.vertices]
        shapely_obstacles.append(ShPolygon(obs_coords))
        for edge in obst.edges:
            lines_for_polygonize.append(LineString([(edge.pt_a[0], edge.pt_a[1]), (edge.pt_b[0], edge.pt_b[1])]))
            
    # Adiciona os Segmentos Verticais (Decomposição)
    for seg in vertical_segments:
        lines_for_polygonize.append(LineString([(seg.pt_a[0], seg.pt_a[1]), (seg.pt_b[0], seg.pt_b[1])]))

    # Polygonize: transforma todas as linhas cruzadas em polígonos fechados
    merged_lines = unary_union(lines_for_polygonize)
    all_polygons = list(polygonize(merged_lines))

    # Filtra os polígonos que são espaço livre (não estão dentro de obstáculos)
    free_cells = []
    for poly in all_polygons:
        # Usa um ponto representativo (interno) para testar se a célula está dentro de algum obstáculo
        repr_pt = poly.representative_point()
        is_inside_obstacle = any(obs.contains(repr_pt) for obs in shapely_obstacles)
        if not is_inside_obstacle:
            free_cells.append(poly)

    # ==========================================
    # 2. CONSTRUÇÃO DO GRAFO E PONTOS MÉDIOS
    # ==========================================
    G = nx.Graph()
    nodes_info = {}

    # Define Início e Fim
    start_pt = ShPoint(2, 2)
    end_pt = ShPoint(45, 45)

    # Encontra o ponto médio esquerdo para cada trapézio livre
    for i, cell in enumerate(free_cells):
        G.add_node(i)
        minx, miny, maxx, maxy = cell.bounds
        
        # Encontra as coordenadas Y na borda esquerda da célula
        left_coords = []
        for x, y in cell.exterior.coords:
            if abs(x - minx) < 1e-4:
                left_coords.append(y)
                
        if left_coords:
            y_medio = (min(left_coords) + max(left_coords)) / 2.0
            nodes_info[i] = ShPoint(minx, y_medio)
        else:
            # Fallback seguro caso o lado esquerdo seja um único vértice
            nodes_info[i] = ShPoint(minx, (miny + maxy) / 2.0)

    # Adiciona Start e End no grafo
    G.add_node('start')
    G.add_node('end')
    nodes_info['start'] = start_pt
    nodes_info['end'] = end_pt

    # Conecta Start e End às células que os contêm
    for i, cell in enumerate(free_cells):
        if cell.contains(start_pt) or cell.distance(start_pt) < 1e-4:
            G.add_edge('start', i)
        if cell.contains(end_pt) or cell.distance(end_pt) < 1e-4:
            G.add_edge('end', i)

    # Conecta células adjacentes (que dividem uma fronteira/linha vertical)
    for i in range(len(free_cells)):
        for j in range(i + 1, len(free_cells)):
            if free_cells[i].touches(free_cells[j]):
                inter = free_cells[i].intersection(free_cells[j])
                # A adjacência só é válida se a fronteira for uma linha com comprimento real
                if inter.geom_type in ['LineString', 'MultiLineString'] and inter.length > 1e-3:
                    G.add_edge(i, j)

    # Executa busca de menor caminho
    try:
        path = nx.shortest_path(G, source='start', target='end')
        path_edges = list(zip(path[:-1], path[1:]))
        print(f"Caminho encontrado! Ordem dos nós: {path}")
    except nx.NetworkXNoPath:
        print("Nenhum caminho encontrado.")
        path_edges = []
        path = []

    # ==========================================
    # 3. PLOTAGEM GERAL
    # ==========================================
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.grid(True, linestyle='--', alpha=0.5)

    # Desenha obstáculos
    for shape in obstacles:
        verts = shape.vertices
        x, y = zip(*verts)
        data = np.transpose(np.array([x, y]))
        ax.fill(data[:, 0], data[:, 1], color='dodgerblue', edgecolor='black', zorder=2)

    # Desenha os segmentos verticais originais
    x_data, y_data = get_plot_data(vertical_segments)
    for j, k in zip(x_data, y_data):
        ax.plot(j, k, color='black', linestyle='--', linewidth=0.8, zorder=1)

    # Desenha as arestas gerais do grafo (Caminhos Possíveis)
    for u, v in G.edges():
        p1, p2 = nodes_info[u], nodes_info[v]
        ax.plot([p1.x, p2.x], [p1.y, p2.y], color='gray', linestyle=':', linewidth=1.5, zorder=3)

    # Desenha nós (Pontos Médios e Finais)
    for n in G.nodes():
        pt = nodes_info[n]
        if n == 'start':
            ax.scatter(pt.x, pt.y, c='green', s=150, marker='o', zorder=5, label='Início')
        elif n == 'end':
            ax.scatter(pt.x, pt.y, c='purple', s=150, marker='X', zorder=5, label='Alvo')
        else:
            ax.scatter(pt.x, pt.y, c='black', s=40, zorder=4, label='Ponto Médio Esquerdo' if n == 0 else "")

    # Destaca o caminho selecionado
    if path_edges:
        for u, v in path_edges:
            p1, p2 = nodes_info[u], nodes_info[v]
            ax.plot([p1.x, p2.x], [p1.y, p2.y], color='red', linewidth=3, zorder=6)

    ax.set_xlim(MIN_X, MAX_X)
    ax.set_ylim(MIN_Y, MAX_Y)
    ax.set(xlabel='X', ylabel='Y', title='Decomposição Trapezoidal com Pathfinding')
    
    # Organiza a legenda (evita rótulos duplicados)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right')

    ax.set_axisbelow(True)
    plt.show()