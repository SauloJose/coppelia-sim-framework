#!/usr/bin/env python3
"""
BRAINBYTE - Gerenciador de Infraestrutura de Robótica
Ponto de entrada principal. Inicia a interface interativa via CLI.
"""

# Adiciona o diretório raiz ao sys.path para garantir que os módulos do projeto sejam encontrados
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from brainbyte.gui.cli import brainGUI
from brainbyte.core import *



if __name__ == "__main__":
    app = brainGUI()
    app.run()





