"""Funções de visualização e plotagem para CoppeliaSim Framework.

Fornece funções padrão para plotar trajetórias 2D e 3D.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def Plot2D(data, x_label, y_label, tamanho_janela=(8, 6), limite_x=None, limite_y=None, title=None):
    """
    Função padrão para plotar gráficos 2D com matplotlib.
    
    Args:
        data (array-like): Array 2D ou lista de pontos [x, y]
        x_label (str): Rótulo do eixo X
        y_label (str): Rótulo do eixo Y
        tamanho_janela (tuple): Tamanho da janela (largura, altura) em polegadas. Default: (8, 6)
        limite_x (tuple): Limites do eixo X (min, max). Default: None (automático)
        limite_y (tuple): Limites do eixo Y (min, max). Default: None (automático)
        title (str): Título do gráfico. Default: None (usa '{y_label} vs {x_label}')
    """
    data = np.array(data)
    
    plt.figure(figsize=tamanho_janela)
    plt.plot(data[:, 0], data[:, 1], 'b-', linewidth=2, label='Trajetória')
    plt.plot(data[0, 0], data[0, 1], 'go', markersize=10, label='Início')
    plt.plot(data[-1, 0], data[-1, 1], 'r*', markersize=15, label='Fim')
    
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title if title is not None else f'{y_label} vs {x_label}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    if limite_x is not None:
        plt.xlim(limite_x)
    if limite_y is not None:
        plt.ylim(limite_y)
    
    plt.show()


def Plot3D(data, x_label, y_label, z_label, tamanho_janela=(8, 6), limite_x=None, limite_y=None, limite_z=None, title=None):
    """
    Função padrão para plotar gráficos 3D com matplotlib.
    
    Args:
        data (array-like): Array 2D ou lista de pontos [x, y, z]
        x_label (str): Rótulo do eixo X
        y_label (str): Rótulo do eixo Y
        z_label (str): Rótulo do eixo Z
        tamanho_janela (tuple): Tamanho da janela (largura, altura) em polegadas. Default: (8, 6)
        limite_x (tuple): Limites do eixo X (min, max). Default: None (automático)
        limite_y (tuple): Limites do eixo Y (min, max). Default: None (automático)
        limite_z (tuple): Limites do eixo Z (min, max). Default: None (automático)
        title (str): Título do gráfico. Default: None (usa '{z_label} vs {x_label} e {y_label}')
    """
    data = np.array(data)
    
    fig = plt.figure(figsize=tamanho_janela)
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(data[:, 0], data[:, 1], data[:, 2], 'b-', linewidth=2, label='Trajetória')
    ax.plot(data[0, 0], data[0, 1], data[0, 2], 'go', markersize=10, label='Início')
    ax.plot(data[-1, 0], data[-1, 1], data[-1, 2], 'r*', markersize=15, label='Fim')
    
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_zlabel(z_label)
    ax.set_title(title if title is not None else f'{z_label} vs {x_label} e {y_label}')
    ax.legend()
    ax.grid(True)
    
    if limite_x is not None:
        ax.set_xlim(limite_x)
    if limite_y is not None:
        ax.set_ylim(limite_y)
    if limite_z is not None:
        ax.set_zlim(limite_z)
    
    plt.show()
