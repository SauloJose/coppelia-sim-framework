import numpy as np
import matplotlib.pyplot as plt


def copPlot(data, x_label, y_label, tamanho_janela=(8, 6), limite_x=None, limite_y=None, title=None):
    """
    Função padrão para plotar gráficos com matplotlib.
    
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
