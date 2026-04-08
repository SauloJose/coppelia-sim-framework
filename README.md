# Curso CoppeliaSim - Framework de Simulação de Robôs

Este projeto fornece um framework para controlar e testar simulações de robôs no CoppeliaSim através da API RemoteAPI via ZMQ.

## Estrutura do Projeto

```
.
├── apps/                    # Aplicações e testes específicos
│   ├── locomocao.py        # Exemplo: teste de locomoção do Pioneer P3DX
│   ├── PionerHouse.py      # Teste em ambiente tipo casa
│   └── teste.py            # Testes gerais
├── core/                    # Núcleo do framework
│   ├── base_app.py         # Classe base para todas as simulações
│   ├── utils.py            # Funções utilitárias (plotagem, etc)
│   └── gui/
│       └── interface.py    # Interface gráfica (opcional)
├── robots/                 # Modelos e definições de robôs
│   ├── bots/              # Comportamentos/drivers de robôs
│   └── sensors/           # Implementação de sensores
│       └── HokuyoSensor.py # Sensor Hokuyo
├── scenes/                 # Arquivos de cena CoppeliaSim (.ttt)
│   ├── house.ttt
│   ├── labirinto.ttt
│   └── locomocao.ttt
├── config.json            # Configurações do projeto
├── requirements.txt       # Dependências Python
└── main.py               # Ponto de entrada principal
```

## Como Funciona

### Ciclo de Vida da Simulação

O framework implementa um ciclo de vida padronizado para todas as simulações:

1. **Conexão**: Conecta ao CoppeliaSim via RemoteAPI ZMQ
2. **Carregamento de Cena**: Carrega o arquivo `.ttt` especificado
3. **Setup**: Executa `setup()` - obtém handles de objetos, sensores e atuadores
4. **Pós-Início**: Executa `post_start()` - testes e diagnósticos após simulação iniciar
5. **Loop Principal**: Executa `loop(t)` repetidamente enquanto `t < sim_time`
6. **Parada**: Executa `stop()` - finalização e limpeza

### Componentes Principais

#### BaseApp (core/base_app.py)

Classe base que fornece toda a infraestrutura de gerenciamento da simulação:

- Gerencia conexão com CoppeliaSim
- Carrega cenas
- Controla o ciclo de simulação
- Permite interrupção via tecla 's'

#### Utils (core/utils.py)

**Sistema de Logging Profissional:**

- `ProfessionalFormatter` - Formata logs com estrutura: `[NÍVEL] [ORIGEM] [TIMESTAMP] mensagem`
- `setup_logger(name, origin_prefix)` - Cria logger padronizado para uso em qualquer arquivo
  
Exemplo de saída:
```
[INFO] [MAIN] [14:32:45] Conectado ao simulador com sucesso
[ERROR] [APP] [14:32:47] Erro ao ler sensor LIDAR
[DEBUG] [APP] [14:32:48] LIDAR [2.45s] Esq: 1.50m | Frente: 2.30m | Dir: 1.20m
```

**Funções de Plotagem:**

- `Plot2D(data, x_label, y_label, tamanho_janela, limite_x, limite_y, title)` - Gráficos 2D com XY
- `Plot3D(data, x_label, y_label, z_label, tamanho_janela, limite_x, limite_y, limite_z, title)` - Gráficos 3D com XYZ

Ambas marcam ponto inicial (verde) e final (vermelho) da trajetória.

## Criando uma Nova Simulação

### Passo 1: Criar a Classe

Crie um arquivo em `apps/` que herde de `BaseApp`:

```python
from core.base_app import BaseApp
from core.utils import setup_logger
import numpy as np

logger = setup_logger(__name__, '[APP]')

class MinhaSimulacao(BaseApp):
    """Descrição do teste/simulação."""
    
    def __init__(self):
        # Especificar cena e tempo de simulação
        super().__init__(scene_file="minha_cena.ttt", sim_time=30.0)
        # Outras inicializações aqui
    
    def setup(self):
        """Executado UMA VEZ antes da simulação começar."""
        pass
    
    def post_start(self):
        """Executado após startSimulation() - testes/diagnósticos (opcional)."""
        pass
    
    def loop(self, t):
        """Executado a cada passo da simulação."""
        pass
    
    def stop(self):
        """Executado após a simulação terminar (opcional)."""
        pass
```

### Passo 2: Implementar os Métodos

#### setup()

**Obrigação**: Obter um handle para cada objeto da cena que será manipulado.

```python
def setup(self):
    logger.info("Configurando simulação...")
    
    # Obtém handle do robô
    self.robot_handle = self.sim.getObject('/Pioneer_p3dx')
    
    # Obtém handles dos motores
    self.left_motor = self.sim.getObject('/Pioneer_p3dx/Pioneer_p3dx_leftMotor')
    self.right_motor = self.sim.getObject('/Pioneer_p3dx/Pioneer_p3dx_rightMotor')
    
    # Valida handles
    if self.robot_handle == -1:
        logger.error("Robot não encontrado na cena!")
        raise RuntimeError("Robot não encontrado na cena!")
    
    logger.debug(f"Handles obtidos: robot={self.robot_handle}")
    
    # Inicializa dados
    self.position = np.array([0, 0, 0])
    self.trajectory = []
```

Métodos úteis do simulador (`self.sim`):

- `getObject(path)` - Obtém handle de um objeto por caminho
- `getObjectPosition(handle, ref_handle)` - Posição do objeto
- `getObjectOrientation(handle, ref_handle)` - Orientação (ângulos de Euler)
- `setJointTargetVelocity(handle, velocity)` - Define velocidade de junta
- `getSimulationTimeStep()` - Obtém dt de simulação

#### loop(t)

**Executado a cada passo de simulação**. Implementar controle e leitura de sensores.

```python
def loop(self, t):
    # Ler sensores
    position = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
    
    # Guardar histórico
    self.trajectory.append(position)
    
    # Controle (ex: velocidade constante)
    self.sim.setJointTargetVelocity(self.left_motor, 0.5)
    self.sim.setJointTargetVelocity(self.right_motor, 0.5)
    
    # Log opcional
    if t % 1.0 < 0.01:  # A cada ~1 segundo
        logger.debug(f"Tempo: {t:.2f}s, Posição: {position}")
```

#### stop()

Executado após a simulação terminar. Ideal para parada segura e análise final.

```python
def stop(self):
    logger.info("Finalizando simulação...")
    
    # Parar motores (IMPORTANTE para segurança)
    try:
        self.sim.setJointTargetVelocity(self.left_motor, 0)
        self.sim.setJointTargetVelocity(self.right_motor, 0)
        logger.info("Motores parados com sucesso")
    except Exception as e:
        logger.error(f"Erro ao parar motores: {e}")
    
    # Análise e plotagem (com Plot2D padrão)
    from core.utils import Plot2D
    Plot2D(self.trajectory, 'X (m)', 'Y (m)', 
           tamanho_janela=(10, 8),
           title="Trajetória do Robô")
```

#### post_start() (Opcional)

Executado logo após `startSimulation()` e ANTES do loop principal. Útil para diagnósticos rápidos e captura de dados iniciais.

```python
def post_start(self):
    """Testes iniciais que requerem simulação já em execução."""
    logger.info("Executando diagnóstico inicial...")
    
    # Enviar pulso de teste
    self.sim.setJointTargetVelocity(self.left_motor, 0.1)
    time.sleep(0.1)
    self.sim.setJointTargetVelocity(self.left_motor, 0)
    
    logger.debug("Diagnóstico concluído com sucesso")
```

### Passo 3: Sistema de Logging Profissional

O framework usa um sistema de logging padronizado sem emojis. Cada arquivo deve importar seu logger assim:

```python
from core.utils import setup_logger

# Em main.py:
logger = setup_logger(__name__, '[MAIN]')

# Em apps/minha_simulacao.py:
logger = setup_logger(__name__, '[APP]')
```

**Saída gerada:**
```
[INFO] [MAIN] [14:32:45] Conectado ao simulador com sucesso
[ERROR] [APP] [14:32:47] Erro ao ler sensor LIDAR
[DEBUG] [APP] [14:32:48] LIDAR [2.45s] Esq: 1.50m | Frente: 2.30m
```

**Níveis de Log:**
- `logger.debug(msg)` - Detalhes técnicos (desabilitado em produção)
- `logger.info(msg)` - Informações importantes
- `logger.warning(msg)` - Avisos (situação anômala mas segura)
- `logger.error(msg)` - Erros recuperáveis
- `logger.critical(msg)` - Falhas críticas

### Passo 4: Criar o Ponto de Entrada

No final do arquivo, crie a função `app()`:

```python
def app():
    """Ponto de entrada esperado por main.py."""
    simulation = MinhaSimulacao()
    simulation.run()
```

### Passo 5: Executar

No `main.py`, importe e execute:

```python
from apps.minha_simulacao import app

if __name__ == "__main__":
    app()
```

Ou execute diretamente:

```bash
python main.py
```

## Métodos Obrigatórios e Opcionais

| Método | Obrigatório | Quando Executado | Descrição |
|--------|-----------|------------------|-----------|
| `setup()` | Sim | Uma vez, antes da simulação | Obter handles, inicializar variáveis |
| `loop(t)` | Sim | A cada passo da simulação | Controle e leitura de sensores |
| `stop()` | Não | Após a simulação terminar | Parada segura, análise, plotagem |
| `post_start()` | Não | Logo após iniciar simulação | Testes e diagnósticos |

## Exemplo Completo

Veja [apps/locomocao.py](apps/locomocao.py) para um exemplo funcional de simulação com:

- Obtenção de handles
- Cinemática (Pioneer P3DX)
- Acúmulo de trajetória
- Plotagem de resultados

## Instalação e Execução

### Requisitos

- Python 3.8+
- CoppeliaSim (versão 4.4.0 ou posterior)
- Dependências listadas em `requirements.txt`

### Setup

```bash
# Instalar dependências
pip install -r requirements.txt

# Executar simulação
python main.py
```

### Interrupção

- Pressionar 's' durante execução: interrompe a simulação
- Ctrl+C: interrupção de emergência

## Otimizações e Boas Práticas

### Sistema de Logging

O projeto usa `ProfessionalFormatter` e `setup_logger()` para garantir:
- Logs consistentes e profissionais (sem emojis)
- Identificação clara de origem (`[MAIN]` vs `[APP]`)
- Timestamps para debug temporal
- Facilita análise de problemas

### Funções de Plotagem

Use `Plot2D()` e `Plot3D()` em vez de matplotlib direto:
```python
# BOA PRATICA:
from core.utils import Plot2D
Plot2D(trajectory, 'X (m)', 'Y (m)', title="Minha Trajetória")

# EVITAR:
import matplotlib.pyplot as plt
plt.plot(...)  # Sem padrão
```

**Ganhos:** Interface consistente, código limpo, manutenção facilitada

### Pré-cálculo de Constantes

Em simulações com loop rápido (60+ ciclos/s), pré-calcule valores em `setup()`:

```python
def setup(self):
    # BOM: Calcular UMA VEZ
    self.idx_frente = self.n_pontos // 2
    self.idx_esq = (3 * self.n_pontos) // 4
    
def loop(self, t):
    # USAR no loop
    valor = dados[self.idx_frente]

# EVITAR: Calcular a cada loop
def loop(self, t):
    idx = int(len(dados) / 2)  # Operação matemática desnecessária
```

**Ganho:** Reduz operações em ~180 ops/s para loops de 60 Hz

### Validação Robusta de Dados

Sempre valide dados de sensores:

```python
# BOM:
data = np.asarray(sensor.getData())
if data is None or data.size == 0:
    return
if data.ndim != 2 or data.shape[1] < 2:
    logger.error(f"Formato inválido: {data.shape}")
    return

# EVITAR:
data = sensor.getData()
if len(data) == 0:  # Pode falhar se data for escalar
    return
```

### Separação de Responsabilidades

Use `post_start()` para inicializações que requerem simulação em execução:

```python
# BOM:
def post_start(self):
    """Captura dados iniciais do sensor"""
    sensor_data = self.sensor.getData()
    
def loop(self, t):
    """Apenas lógica de controle"""
    
# EVITAR:
def loop(self, t):
    if self._first_exec:
        sensor_data = self.sensor.getData()  # Flag desnecessária
    self._first_exec = False
```

## Troubleshooting

### "Código congela ao conectar"

O CoppeliaSim está fechado. Abra a aplicação antes de executar.

### "Handle não encontrado (retorna -1)"

Verifique:
1. O caminho do objeto no CoppeliaSim (deve começar com `/`)
2. A cena está carregada corretamente
3. O nome do objeto/robô no arquivo `.ttt`

### "Erro de comunicação ZMQ"

Verifique se a RemoteAPI está ativada no CoppeliaSim:
- Menu: `Tools > Remote API server` deve estar **ligado**

### "TypeError: len() of unsized object"

Sensor retornou dados em formato inválido. Verifique:
1. Sensor conectado corretamente no CoppeliaSim
2. Validações em `loop()` tratam dados escalar/vazio
3. Logs indicam `shape_do_sensor` esperado

## Histórico de Melhorias

### v1.1 - Sistema de Logging Profissional
- Removidos todos os emojis
- Criado `ProfessionalFormatter` com estrutura padronizada
- Função `setup_logger()` para uso consistente
- Logs rastreáveis com timestamp e origem

### v1.0 - Framework Base
- Classe `BaseApp` com ciclo de vida completo
- Métodos `setup()`, `loop(t)`, `stop()`, `post_start()`
- Conexão RemoteAPI com CoppeliaSim
- Funções `Plot2D()` e `Plot3D()` reutilizáveis

## Contribuições

Para adicionar novas simulações:

1. Crie arquivo em `apps/seu_teste.py`
2. Herde de `BaseApp`
3. Implemente `setup()` e `loop(t)`
4. Use `setup_logger()` para logging
5. Adicione função `app()` como ponto de entrada
6. Teste com `python main.py`

Mantenha a estrutura do projeto organizada e siga o padrão de logging profissional.


