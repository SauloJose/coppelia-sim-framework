# Arquitetura do Framework CoppeliaSim

## Visão Geral

O Framework CoppeliaSim é um pacote Python profissional projetado para simplificar o controle de robôs e a simulação usando a API remota ZMQ do CoppeliaSim. Ele fornece:

- **BaseApp**: Uma classe base que gerencia o ciclo de vida da simulação.
- **SimulationBridge**: Uma ponte de comunicação de altíssima performance baseada em lotes (Batch Dataflow) e CBOR.
- **Logging Profissional**: Sistema de logs padronizado e limpo.
- **Ferramentas de Visualização**: Funções integradas para plotagem 2D e 3D.
- **Melhores Práticas**: Exemplos estruturados e arquitetura orientada a objetos.

## Estrutura de Diretórios

```text
coppelia_sim_framework/
├── __init__.py              # Exportações do pacote
├── core/
│   ├── __init__.py
│   ├── base_app.py          # Classe BaseApp - ciclo de vida da simulação
│   ├── bridge.py            # SimulationBridge - comunicação ZMQ/CBOR em lote
│   └── logging.py           # ProfessionalFormatter e setup_logger()
├── utils/
│   ├── __init__.py
│   └── plotting.py          # Funções de visualização Plot2D() e Plot3D()
├── sensors/                 # Implementações de sensores (ex: LDS_02)
├── robots/                  # Definições de robôs (ex: TurtleBot)
└── gui/                     # Componentes de interface de usuário

projects/                    # Aplicações de exemplo
├── __init__.py
├── exemple/
│   ├── __init__.py
│   ├── exemple.py           # Aplicação principal
│   └── scene.ttt            # Cena do CoppeliaSim
tests/                       # Testes unitários e de integração
├── __init__.py
└── test_framework.py

docs/                        # Documentação
├── ARCHITECTURE.md          # Este arquivo
├── API.md                   # Referência da API
├── CONTRIBUTING.md          # Diretrizes de contribuição
└── TUTORIALS.md             # Tutoriais passo-a-passo

scripts/                     # Scripts utilitários
config/                      # Arquivos de configuração

main.py                      # Menu interativo para rodar os exemplos
setup.py                     # Configuração para instalação via pip
pyproject.toml               # Metadados modernos do projeto Python
requirements-dev.txt         # Dependências de desenvolvimento
README.md                    # Documentação principal
.gitignore                   # Regras de ignorar do Git
```

## Arquitetura de Comunicação em Lote (Batch Dataflow)

A característica mais poderosa deste framework é a sua ponte de comunicação (`SimulationBridge`). Em vez de fazer dezenas de chamadas de rede isoladas por frame para ler sensores e enviar velocidades (o que causaria um enorme gargalo de rede), o sistema funciona com o conceito de **Servidor de Fluxo de Dados em Lote Síncrono**.

1. **Handshake e Pré-Cache (`INIT`)**: Antes da simulação rodar, o Python envia uma lista de todos os caminhos dos objetos que ele deseja atuar ou monitorar. O CoppeliaSim (via script Lua) faz a busca dessas IDs ("handles") uma única vez e os guarda na memória.
2. **Buffer de Comandos (`queue_...`)**: Durante o loop de controle, o robô não envia comandos imediatamente. Métodos como `queue_velocity` ou `queue_position` apenas adicionam as intenções de movimento a um "carrinho de compras" virtual no Python.
3. **Passo Síncrono (`step`)**: Uma vez por frame, a ponte pega todos os comandos acumulados e os envia em um **único pacote binário ultrarrápido (CBOR)** via ZeroMQ.
4. **Captura Global (Sensores)**: O CoppeliaSim aplica os comandos, avança exatamente 1 frame da física e empacota o estado de todos os sensores solicitados (incluindo nuvens de pontos LiDAR, matrizes e telemetria) em um pacote de resposta.
5. **Acesso com Atraso Zero**: Quando as classes de sensores (como a `LDS_02`) precisam ler os dados, elas não acessam a rede. Elas simplesmente consultam o dicionário `latest_state` salvo na memória local do Python, garantindo tempo de execução na casa dos microssegundos.

## Responsabilidades dos Módulos

### core.base_app

**Propósito**: Gerenciar o ciclo de vida da simulação.

- Lida com a inicialização da `SimulationBridge`.
- Carrega as cenas.
- Gerencia o tempo e os passos de simulação (Stepping).
- Fornece ganchos (hooks): `setup()`, `post_start()`, `loop(t)`, `stop()`.

### core.bridge

**Propósito**: Ponte de baixo nível entre o Python e o motor Lua do CoppeliaSim.

- Gerencia o socket ZMQ (REQ/REP).
- Enfileira comandos (`queue_velocity`, `queue_position`, `queue_command`).
- Transforma dados usando o formato binário CBOR para suportar matrizes Numpy nativas (LiDAR e Visão) sem erros de decodificação.

### core.logging

**Propósito**: Sistema de logging profissional.

- Formato padronizado: `[LEVEL] [ORIGIN] [TIMESTAMP] mensagem`.
- Evita que travamentos silenciosos passem despercebidos.

## Fluxo de Execução

```text
┌─────────────────────────────────────────┐
│  main.py                                │
│  (Menu interativo de exemplos)          │
└──────────────────┬──────────────────────┘
                   │
                   └─→ turtlebot_example.py
                       └→ TurtleBotApp(BaseApp)
                           └→ app() aciona .run()

Quando .run() é chamado:
1. Carrega a cena (.ttt).
2. Conecta ao ZMQ e envia modo Síncrono.
3. Chama setup() [VOCÊ SOBRESCREVE ISTO] - Inicia sensores e robôs.
4. Bridge envia pacote INIT (Handshake de handles).
5. Inicia a Simulação.
6. Chama post_start() [VOCÊ SOBRESCREVE ISTO].
7. Loop principal:
   - Verifica interrupção pelo usuário.
   - Chama loop(t) [VOCÊ SOBRESCREVE ISTO] - Lógica do usuário agrupa comandos.
   - Bridge.step() - Envia lote CBOR, avança física, recebe sensores atualizados.
8. Ao sair:
   - Chama stop() [VOCÊ SOBRESCREVE ISTO].
   - Encerra simulação e fecha portas de rede.
```

## Padrão de Logging

Todos os módulos utilizam o sistema de log profissional:

```python
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[APP]')  # ou '[MAIN]' no main.py

logger.info("Starting simulation...")      # [INFO] [APP] Starting simulation...
logger.warning("Sensor LiDAR falhou")      # [WARNING] [APP] ...
logger.error("Falha ao conectar ZMQ")      # [ERROR] [APP] ...
```

## Melhores Práticas Implementadas

1. **Eficiência de Rede**: Paradigma de lote (Batch) evita comunicação "chatty" (conversadora) entre os processos.
2. **Separação de Preocupações**: A `BaseApp` lida com o tempo e os estados; a `SimulationBridge` lida com os bytes da rede.
3. **Formatos Binários**: Uso do `cbor2` no lugar do JSON para impedir problemas de codificação Unicode ao ler dados volumosos (Nuvens de Pontos/Câmeras).
4. **Isolamento de Estado**: Os dados reais vs. referências calculadas estão separados.
5. **Tratamento de Erros**: Blocos Try-except focados e fechamento gracioso (graceful shutdown) nos blocos `finally`.

## Adicionando um Novo Exemplo

1. Crie um novo arquivo em `projects/` (ex: `meu_teste.py`).
2. Herde a classe `BaseApp`.
3. Implemente `setup()`, `loop(t)` e `stop()`.
4. Defina uma função `app()` que instancie e rode seu exemplo.
5. O exemplo estará pronto para ser testado via `main.py`.

Modelo base:

```python
from coppelia_sim_framework import BaseApp, setup_logger

logger = setup_logger(__name__, '[APP]')

class MeuTeste(BaseApp):
    def __init__(self):
        super().__init__(scene_file="my_scene.ttt", sim_time=30.0)
    
    def setup(self):
        logger.info("Configurando robôs e sensores...")
        # Instancie o Turtlebot, adicione o LDS_02...
    
    def loop(self, t):
        # Lógica de controle, algoritmos PID, processamento do Lidar
        pass
    
    def stop(self):
        logger.info("Limpando ambiente...")
        # Salve resultados, gere gráficos Matplotlib

def app():
    teste = MeuTeste()
    teste.run()

if __name__ == "__main__":
    app()
```

## Extensões Futuras

- **SLAM e Mapeamento**: Implementar integração direta das Nuvens de Pontos processadas do cache.
- **Novos Robôs**: Adicionar defnições pré-configuradas de robôs (Omnidirecionais, Braços Industriais).
- **Controle Avançado**: Módulos de planejamento de trajetória em C-Space nativos.
- **CI/CD**: Integração em pipelines contínuos para testes automáticos de regressão de simulador.

## Referências

- [Documentação do CoppeliaSim](https://www.coppeliarobotics.com/helpFiles/)
- [ZMQ Remote API e Sincronismo](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteAPIOverview.htm)
- [Documentação do Padrão CBOR](https://cbor.io/)
```