#!/usr/bin/env python3
"""
BRAINBYTE - Gerenciador de Infraestrutura de Robótica
Ponto de entrada principal. Inicia a interface interativa via CLI.
"""

import sys
import json
import subprocess
from pathlib import Path

# Configuração de caminhos base
BASE_DIR = Path(__file__).parent
CONFIG_FILE = BASE_DIR / "config.json"
REQUIREMENTS_FILE = BASE_DIR / "requirements.txt"

def verify_and_install_dependencies():
    """
    Verifica no config.json se as dependências atuais já foram instaladas.
    Se não foram, ou se o requirements.txt mudou, faz a instalação.
    """
    if not REQUIREMENTS_FILE.exists():
        return  # Se não tem requirements.txt, ignora a verificação

    # Lê as dependências e versões que o projeto exige atualmente
    with open(REQUIREMENTS_FILE, 'r') as f:
        # Remove espaços e ignora linhas em branco ou comentários
        current_reqs = [line.strip() for line in f if line.strip() and not line.startswith('#')]

    # Lê o config.json (cria os dados iniciais se não existir ou estiver vazio)
    config_data = {}
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, 'r') as f:
                config_data = json.load(f)
        except json.JSONDecodeError:
            config_data = {}

    # Pega o que está salvo no config (o que já foi instalado antes)
    saved_reqs = config_data.get("installed_dependencies", [])

    # Se a lista de exigências for diferente do que está salvo, precisamos instalar
    if current_reqs != saved_reqs:
        print("\n[INFO] Dependências novas ou desatualizadas detectadas. Instalando pacotes...")
        try:
            # Chama o pip internamente usando o executável atual do Python
            subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", str(REQUIREMENTS_FILE)])
            print("[INFO] Instalação concluída com sucesso!\n")
            
            # Atualiza o config.json com as dependências que acabaram de ser instaladas
            config_data["installed_dependencies"] = current_reqs
            with open(CONFIG_FILE, 'w') as f:
                json.dump(config_data, f, indent=4)
                
        except subprocess.CalledProcessError as e:
            print(f"\n[ERRO FATAL] Falha ao instalar dependências. Verifique sua conexão ou o requirements.txt.")
            print(f"Detalhe do erro: {e}")
            sys.exit(1)  # Encerra o script pois não tem como rodar sem as dependências
    else:
        # Tudo certo, nada a fazer
        pass

# Garantimos que tudo está instalado!
verify_and_install_dependencies()

# Adiciona o diretório ao path
sys.path.insert(0, str(BASE_DIR))

# Agora sim fazemos os imports do projeto com segurança
from brainbyte.gui.cli import brainGUI
from brainbyte.core import *


if __name__ == "__main__":
    app = brainGUI()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupção pelo usuário detectada. Saindo graciosamente...")
        # Opcional: forçar o stop se você tiver a instância da simulação acessível
    except Exception as e:
        print(f"Ocorreu um erro inesperado: {e}")