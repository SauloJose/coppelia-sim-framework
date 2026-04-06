"""Utilitários e classe base para aplicações que controlam o CoppeliaSim via ZMQ.

`BaseApp` gerencia o ciclo de vida comum a testes/experimentos:
- carregar cena (.ttt)
- configurar modo síncrono
- executar `setup()` (uma vez)
- iterar `loop(t)` até `sim_time` ou interrupção

Comentários:
- Esta camada separa a lógica de controle (nos testes) da mecânica de execução/controle
  da simulação, facilitando testes e reuso.
"""

import os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# A biblioteca `keyboard` permite detectar pressionamento de teclas sem bloquear.
# Observação: em alguns sistemas ela exige privilégios/admin e pode não ser ideal
# para ambientes headless. Alternativas: sinalização via sim ou socket externo.
import keyboard
import sys 
import time 

class BaseApp:
    """Classe base que fornece o ciclo de vida mínimo de uma aplicação de simulação.

    Subclasses devem sobrescrever `setup()` e `loop(t)` para implementar o teste.
    """
    def __init__(self, scene_file=None, sim_time=10.0):
        self.scene_file = scene_file
        self.sim_time = sim_time
        
        # Avisa o usuário ANTES do código travar
        print("\n⏳ Tentando conectar ao motor do CoppeliaSim...")
        print("👉 (Se o terminal travar nesta mensagem, o simulador está FECHADO. Abra o CoppeliaSim!)")
        
        try:

            # É aqui que o código "congela" se o simulador estiver fechado
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')

            # Parar a simulação se estiver executando
            initial_sim_state = self.sim.getSimulationState()
            if initial_sim_state != 0:
                self.sim.stopSimulation()
                time.sleep(1)


            # Se passou da linha de cima, deu certo!
            print("✅ Conectado ao simulador com sucesso!\n")
            
        except Exception as e:
            print("\n❌ ERRO DE CONEXÃO: Não foi possível estabelecer comunicação.")
            print(f"Detalhes: {e}")
            sys.exit(1)

    def run(self):
        """Método principal que gerencia o ciclo de vida da simulação."""
        # 1. Carrega a cena (se especificada)
        if self.scene_file:
            # O CoppeliaSim exige caminhos ABSOLUTOS para carregar cenas
            scene_path = os.path.abspath(f"scenes/{self.scene_file}")
            if not os.path.exists(scene_path):
                raise FileNotFoundError(f"Cena não encontrada: {scene_path}")
            
            print(f"Carregando cena: {self.scene_file}...")
            self.sim.loadScene(scene_path)

        # 2. Configura modo síncrono
        self.sim.setStepping(True)

        # 3. Executa o setup específico do teste (definido pelo filho)
        self.setup()

        # 4. Inicia a simulação
        print("Iniciando simulação...")
        self.sim.startSimulation()

        # 5. Loop principal no tempo programado
        try:
            while (t := self.sim.getSimulationTime()) < self.sim_time:
                
                # Verificação de interrupção rápida pelo usuário.
                # Pressionar 's' interrompe a simulação imediatamente.
                if keyboard.is_pressed('s'):
                    print(f"\n⚠️ Simulação interrompida pelo usuário no tempo {t:.2f}s.")
                    break
                
                self.loop(t)
                
                # Avança um passo no simulador
                self.sim.step()
                
        except KeyboardInterrupt:
            print("\n⚠️ Simulação interrompida manualmente no terminal.")
            
        finally:
            # 6. Para a simulação independentemente de erro ou sucesso
            print("Parando simulação...")
            self.sim.stopSimulation()

    # ==========================================
    # MÉTODOS PARA SOBRESCREVER NAS CLASSES FILHAS
    # ==========================================
    def setup(self):
        """Executado uma vez ANTES da simulação começar (ideal para pegar handles)."""
        pass

    def loop(self, t):
        """Executado a cada passo da simulação (ideal para controle e sensores)."""
        pass