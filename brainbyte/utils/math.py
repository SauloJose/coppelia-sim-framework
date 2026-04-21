import numpy as np
from numba import njit 
import matplotlib.pyplot as plt
import numpy as np
import time

#Método de ortogonalização 
@njit(fastmath=True)
def gram_schmidt(V: np.ndarray):
    """
    Recebe uma matriz V (vetores genéricos nas colunas)
    Retorna uma matriz U (vetores ortonormais nas colunas).
    """
    M = V.shape[0] # Linhas (dimensão do espaço, ex: R3)
    N = V.shape[1] # Colunas (quantidade de vetores)

    # Crio a matriz vazia para os vetores ortogonais
    U = np.zeros((M,N))

    for i in range(N):
        # Pega o i-ésimo vetor original
        v_atual = V[:,i].copy()

        # subtrai a projeção de todos os vetores ortogonais já calculados
        for j in range(i):
            u_anterior = U[:,j]

            #formula de projeção
            numerador = np.dot(v_atual,u_anterior)
            denominador = np.dot(u_anterior,u_anterior)

            # Multiplica pela sombra gerada e subtrai
            sombra = (numerador/denominador) * u_anterior
            v_atual = v_atual - sombra 

        # normaliza o vetor 
        norma = np.sqrt(np.dot(v_atual,v_atual))

        # Evita divisão por zero caso o vetor seja nulo (linearmente dependente)
        if norma > 1e-10:
            U[:, i] = v_atual / norma
        else:
            U[:, i] = v_atual
            
    return U


#definindo função que retorna a matriz de transformação de bases
@njit(fastmath=True)
def T_base(A, B):
    """
    Método para retornar a matriz de transformação de bases.
    
    ATENÇÃO: A base final (B) DEVE ser ortogonal previamente. 
    
    Parâmetros:
    start_base: np.array (Base inicial m vetores do R^n) (dimensão nxn) onde
    end_base: np.array (Base final) (dimensão mxm)

    Retorno:
    T_base: np.array (Matriz de transformação que leva de start_base para end_base, sendo
    portanto, uma matrix m x n)
    """
    # puxando tamanho
    N = A.shape[1]
    M = B.shape[1]

    # Gerando matriz
    T_AB = np.zeros((M,N)) 
    
    for i in range(M):
        bi = np.ascontiguousarray(B[:,i])

        norm_bi = np.dot(bi, bi)

        for j in range(N):
            aj = np.ascontiguousarray(A[:,j])

            proj_ab = np.dot(aj,bi)

            T_AB[i,j] = proj_ab / norm_bi
        
    return T_AB

# Forma rápida de gerar uma matriz de transformação de basesa
def T_base_geral(A: np.ndarray, B: np.ndarray):
    """
    Calcula a matriz de mudança de base da Base A para a Base B.
    Assuma que as colunas das matrizes representam os vetores das bases.
    
    Parâmetros:
    A: np.ndarray (Matriz cujas colunas são os vetores da base de origem)
    B: np.ndarray (Matriz cujas colunas são os vetores da base de destino)
    """
    # np.linalg.solve resolve a equação B * T = A de forma otimizada
    # É muito mais rápido e numericamente estável do que fazer inv(B) @ A
    T_AB = np.linalg.solve(B, A)
    
    return T_AB

# Matrizes de rotação
@njit(cache=True)
def Rx(theta):
    """Matriz de rotação 3x3 em torno do eixo X."""
    c = np.cos(theta)
    s = np.sin(theta)
    
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c,  -s ],
        [0.0, s,   c ]
    ])

@njit(cache=True)
def Ry(theta):
    """Matriz de rotação 3x3 em torno do eixo Y."""
    c = np.cos(theta)
    s = np.sin(theta)
    
    return np.array([
        [ c,  0.0, s ],
        [0.0, 1.0, 0.0],
        [-s,  0.0, c ]
    ])

@njit(cache=True)
def Rz(theta):
    """Matriz de rotação 3x3 em torno do eixo Z."""
    c = np.cos(theta)
    s = np.sin(theta)
    
    return np.array([
        [ c, -s,  0.0],
        [ s,  c,  0.0],
        [0.0, 0.0, 1.0]
    ])

# Constroi matriz de transformação em coordenadas homogêneas
def T_homo(R, p):
    """Constrói matriz homogênea 4x4 a partir de R (3x3) e p (3,)."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


