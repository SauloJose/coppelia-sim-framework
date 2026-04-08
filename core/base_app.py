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
import logging

logger = logging.getLogger(__name__)

class BaseApp:
    """Classe base que fornece o ciclo de vida mínimo de uma aplicação de simulação.

    Subclasses devem sobrescrever `setup()` e `loop(t)` para implementar o teste.
    """
    def __init__(self, scene_file=None, sim_time=10.0):
        self.scene_file = scene_file
        self.sim_time = sim_time
        
        # Avisa o usuário ANTES do código travar
        logger.info("\n⏳ Tentando conectar ao motor do CoppeliaSim...")
        logger.info("👉 (Se o terminal travar nesta mensagem, o simulador está FECHADO. Abra o CoppeliaSim!)")
        
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
            logger.info("✅ Conectado ao simulador com sucesso!\n")
            
        except Exception as e:
            logger.exception("\n❌ ERRO DE CONEXÃO: Não foi possível estabelecer comunicação.")
            logger.error(f"Detalhes: {e}")
            sys.exit(1)

    def run(self):
        """Método principal que gerencia o ciclo de vida da simulação."""
        # Carrega a cena (se especificada)
        if self.scene_file:
            # O CoppeliaSim exige caminhos ABSOLUTOS para carregar cenas
            scene_path = os.path.abspath(f"scenes/{self.scene_file}")
            if not os.path.exists(scene_path):
                raise FileNotFoundError(f"Cena não encontrada: {scene_path}")
            
            logger.info(f"Carregando cena: {self.scene_file}...")
            self.sim.loadScene(scene_path)   #Carregar uma cena, arquivo .ttt

        # Configura modo síncrono
        self.client.setStepping(True)

        # Executa o setup específico do teste (definido pelo filho)
        self.setup()

        # Inicia a simulação
        logger.info("Iniciando simulação...")
        self.sim.startSimulation()

        # Hook executado logo APÓS o startSimulation() e ANTES do loop principal.
        # Subclasses podem sobrescrever `post_start()` para executar ações
        # que precisam da simulação já em execução (ex.: diagnóstico rápido).
        try:
            self.post_start()
        except Exception:
            logger.exception("Erro em post_start()")

        # Loop principal no tempo programado
        try:
            while (t := self.sim.getSimulationTime()) < self.sim_time:
                
                # Verificação de interrupção rápida pelo usuário.
                # Pressionar 's' interrompe a simulação imediatamente.
                if keyboard.is_pressed('s'):
                    logger.warning(f"\n⚠️ Simulação interrompida pelo usuário no tempo {t:.2f}s.")
                    break
                
                self.loop(t)
                
                # Avança um passo no simulador
                self.client.step()
                
        except KeyboardInterrupt:
            logger.warning("\n⚠️ Simulação interrompida manualmente no terminal.")
            
        finally:
            # Para a simulação independentemente de erro ou sucesso
            logger.info("Parando simulação...")

            self.stop() #Procedimento para quando parar a simulação
            self.sim.stopSimulation()

    # ==========================================
    # MÉTODOS PARA SOBRESCREVER NAS CLASSES FILHAS
    # ==========================================
    def setup(self):
        """Executado uma vez ANTES da simulação começar (ideal para pegar handles)."""
        pass

    def post_start(self):
        """Hook executado imediatamente após `startSimulation()`.

        Subclasses podem sobrescrever este método para executar rotinas que
        exigem que a simulação já esteja em execução (ex.: checagens rápidas,
        diagnóstico de atuadores, etc.). O padrão é não fazer nada.
        """
        return

    def loop(self, t): 
        """Executado a cada passo da simulação (ideal para controle e sensores)."""
        pass

    def stop(self):
        """Executado após a finalização da simulação"""
        pass 