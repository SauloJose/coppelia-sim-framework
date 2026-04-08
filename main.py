"""Painel interativo para listar e executar testes na pasta `apps`.

Expectativa:
- Cada arquivo em `apps/` deve expor uma função `app()` que instancia e executa o teste.

Uso:
- Rode este arquivo com `python main.py` e selecione um teste pelo número.
"""
import importlib
from pathlib import Path
from core.utils import setup_logger

logger = setup_logger(__name__, '[MAIN]')

def listar_testes():
    """Lê a pasta 'apps' e retorna uma lista com o nome dos arquivos de teste."""
    pasta_apps = Path("apps")
    testes = []
    
    # Verifica se a pasta existe antes de tentar ler
    if not pasta_apps.exists() or not pasta_apps.is_dir():
        logger.warning("Aviso: A pasta 'apps' não foi encontrada.")
        return testes
        
    # Procura todos os arquivos que terminam com .py na pasta apps
    for arquivo in pasta_apps.glob("*.py"):
        nome_arquivo = arquivo.stem  # Pega só o nome (ignora o .py)
        
        # Ignora o __init__.py e arquivos ocultos
        if nome_arquivo != "__init__" and not nome_arquivo.startswith("."):
            testes.append(nome_arquivo)
            
    # Retorna em ordem alfabética para o menu ficar organizado
    return sorted(testes)

def main():
    """Loop interativo principal.

    - Lista módulos em `apps/` (arquivos .py, exceto __init__).
    - Apresenta menu e importa dinamicamente o módulo escolhido.
    - Procura uma função `app()` no módulo para executar o teste.
    """

    # 1. Lê a pasta dinamicamente
    lista_testes = listar_testes()
    
    if not lista_testes:
        logger.error("Nenhum teste encontrado na pasta 'apps/'.")
        return

    # 2. Monta o Menu
    print("\n" + "="*40)
    print(" MENU DO SIMULADOR COPPELIASIM ")
    print("="*40)
    print("Selecione o teste que deseja executar:\n")
    
    for i, nome_teste in enumerate(lista_testes):
        print(f"[{i + 1}] - {nome_teste}")
        
    print("[0] - Sair")
    print("-" * 40)
    
    # 3. Loop de interação
    while True:
        escolha = input("Digite o número da opção: ")
        
        if escolha == '0':
            print("Encerrando o painel. Até logo!")
            break
            
        if escolha.isdigit():
            indice = int(escolha) - 1
            
            if 0 <= indice < len(lista_testes):
                nome_selecionado = lista_testes[indice]
                logger.info(f"Iniciando o teste: {nome_selecionado}")  
                
                # 4. Importação e Execução Dinâmica
                try:
                    # Importa o módulo escolhido (Ex: from apps import teste_cinematica)
                    modulo = importlib.import_module(f"apps.{nome_selecionado}")
                    
                    # Verifica se você realmente criou a função 'app()' dentro do arquivo
                    if hasattr(modulo, 'app'):
                        modulo.app()
                    else:
                        logger.error(f"Arquivo '{nome_selecionado}.py' não possui função 'app()'")
                        
                except ImportError as e:
                    logger.error(f"Erro ao tentar carregar o teste '{nome_selecionado}': {e}")
                except Exception as e:
                    logger.error(f"Erro crítico ao executar o teste '{nome_selecionado}': {type(e).__name__}: {e}")
                    
                break # Encerra o menu após rodar o teste
            else:
                logger.error("Número fora da lista. Tente novamente.")
        else:
            logger.error("Por favor, digite apenas números válidos.")

if __name__ == '__main__':
    main()