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

Funções utilitárias para plotagem de dados:

- `Plot2D(data, x_label, y_label, ...)` - Gráficos 2D com XY
- `Plot3D(data, x_label, y_label, z_label, ...)` - Gráficos 3D com XYZ

## Criando uma Nova Simulação

### Passo 1: Criar a Classe

Crie um arquivo em `apps/` que herde de `BaseApp`:

```python
from core.base_app import BaseApp
import logging
import numpy as np

logger = logging.getLogger(__name__)

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
        raise RuntimeError("Robot não encontrado na cena!")
    
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

Executado após a simulação terminar. Ideal para limpeza e análise final.

```python
def stop(self):
    logger.info("Finalizando simulação...")
    
    # Parar motores (segurança)
    self.sim.setJointTargetVelocity(self.left_motor, 0)
    self.sim.setJointTargetVelocity(self.right_motor, 0)
    
    # Plotar trajetória
    from core.utils import Plot2D
    Plot2D(self.trajectory, 'X (m)', 'Y (m)', title="Trajetória do Robô")
```

#### post_start() (Opcional)

Executado logo após `startSimulation()`. Útil para diagnósticos rápidos.

```python
def post_start(self):
    """Teste rápido: enviar um pulso de controle para validar conexão."""
    logger.debug("Testando motores...")
    self.sim.setJointTargetVelocity(self.left_motor, 0.1)
    time.sleep(0.1)
    self.sim.setJointTargetVelocity(self.left_motor, 0)
```

### Passo 3: Criar o Ponto de Entrada

No final do arquivo, crie a função `app()`:

```python
def app():
    """Ponto de entrada esperado por main.py."""
    simulation = MinhaSimulacao()
    simulation.run()
```

### Passo 4: Executar

No `main.py`, importe e execute:

```python
from apps.minha_simulacao import app

if __name__ == "__main__":
    app()
```

Ou execute diretamente:

```bash
python -m apps.minha_simulacao
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

## Contribuições

Adicione novas simulações seguindo o padrão de herança de `BaseApp`. Mantenha a estrutura do projeto organizada.

