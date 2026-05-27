import numpy as np
from shapely.geometry import Polygon, box

class Cell:
    def __init__(self, row, col, cx, cy, cell_size):
        self.row = row          # Índice da linha na matriz
        self.col = col          # Índice da coluna na matriz
        self.cx = cx            # Coordenada X real do centro da célula
        self.cy = cy            # Coordenada Y real do centro da célula
        self.size = cell_size   # Tamanho lateral da célula
        self.occupied = 0       # 0 = Livre, 1 = Obstáculo

    def __repr__(self):
        return f"Cell({self.row},{self.col}) -> Pos:({self.cx:.2f}, {self.cy:.2f}) | Occupied: {self.occupied}"


class GridMap:
    def __init__(self, x_min=-7.0, x_max=7.0, y_min=-7.0, y_max=7.0, cell_size=0.2):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.cell_size = cell_size

        # Calcula dimensões da matriz
        self.linhas = int(round((y_max - y_min) / cell_size))
        self.colunas = int(round((x_max - x_min) / cell_size))
        
        # Inicializa matriz binária para o A*
        self.matrix = np.zeros((self.linhas, self.colunas), dtype=int)
        
        # Cria a tabela de objetos Cell
        self.cells = []
        self._init_cells()

    def _init_cells(self):
        """Inicializa os objetos de células calculando seus centros reais."""
        self.cells = []
        for i in range(self.linhas):
            row_cells = []
            for j in range(self.colunas):
                # Calcula o centro geométrico da célula (ideal para o A*)
                cx = self.x_min + (j * self.cell_size) + (self.cell_size / 2.0)
                cy = self.y_min + (i * self.cell_size) + (self.cell_size / 2.0)
                row_cells.append(Cell(row=i, col=j, cx=cx, cy=cy, cell_size=self.cell_size))
            self.cells.append(row_cells)

    def world_to_grid(self, x, y):
        """Converte uma coordenada real (X, Y) do mundo para índices (row, col) da matriz."""
        col = int((x - self.x_min) / self.cell_size)
        row = int((y - self.y_min) / self.cell_size)
        # Garante que os índices fiquem dentro dos limites da matriz
        col = np.clip(col, 0, self.colunas - 1)
        row = np.clip(row, 0, self.linhas - 1)
        return int(row), int(col)

    def grid_to_world(self, row, col):
        """Converte índices da matriz (row, col) de volta para o centro real (X, Y)."""
        cell = self.cells[row][col]
        return cell.cx, cell.cy

    def build_grid(self, obstacles_data):
        """Preenche a matriz com base nos polígonos recebidos do simulador."""
        # Limpa o grid atual
        self.matrix.fill(0)
        
        # Transforma os dicionários de obstáculos em polígonos Shapely válidos
        shapely_obstacles = []
        for obs in obstacles_data:
            if 'corners' in obs and len(obs['corners']) >= 3:
                shapely_obstacles.append(Polygon(obs['corners']))

        # Varre as células verificando colisões
        for i in range(self.linhas):
            for j in range(self.colunas):
                cell = self.cells[i][j]
                
                # Cria a caixa de colisão da célula
                c_xmin = cell.cx - (self.cell_size / 2.0)
                c_xmax = cell.cx + (self.cell_size / 2.0)
                c_ymin = cell.cy - (self.cell_size / 2.0)
                c_ymax = cell.cy + (self.cell_size / 2.0)
                cell_box = box(c_xmin, c_ymin, c_xmax, c_ymax)

                # Se a célula interceptar qualquer obstáculo inflado do simulador
                if any(cell_box.intersects(obs) for obs in shapely_obstacles):
                    cell.occupied = 1
                    self.matrix[i, j] = 1
                else:
                    cell.occupied = 0

    def get_neighbors(self, row, col, allow_diagonal=True):
        """Retorna os vizinhos válidos livres (útil para o algoritmo A*)."""
        neighbors = []
        # Deslocamentos: (d_row, d_col)
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Cardinais
        if allow_diagonal:
            moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)] # Diagonais

        for dr, dc in moves:
            r, c = row + dr, col + dc
            if 0 <= r < self.linhas and 0 <= c < self.colunas:
                if self.matrix[r, c] == 0: # Se estiver livre
                    neighbors.append((r, c))
        return neighbors