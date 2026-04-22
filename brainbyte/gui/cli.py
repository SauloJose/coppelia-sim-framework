import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap
from brainbyte.gui.auxF import * 

from brainbyte.utils.logging import *  # Certifique-se de que este módulo existe
from brainbyte.core.paths import *
import traceback

class brainGUI:
    def __init__(self):
        self.logger = setup_logger(__name__, '[BRAINBYTE]',log_file=LOG_BRAIN_FILE)
        self.examples_folder = Path("examples")
        self.examples_list = []
        # Configurações padrão
        self.config = {
            'cli_commands': False,
            'ros_connection': False,
            'udp_connection': False 
        }

    @staticmethod
    def banner():
        """Exibe o banner ASCII do BRAINBYTE com alinhamento consistente."""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        term_width = shutil.get_terminal_size().columns
        
        title_lines = [
            "██████╗ ██████╗  █████╗ ██╗███╗   ██╗██████╗ ██╗   ██╗████████╗███████╗",
            "██╔══██╗██╔══██╗██╔══██╗██║████╗  ██║██╔══██╗╚██╗ ██╔╝╚══██╔══╝██╔════╝",
            "██████╔╝██████╔╝███████║██║██╔██╗ ██║██████╔╝ ╚████╔╝    ██║   ███████╗",
            "██╔══██╗██╔══██╗██╔══██║██║██║╚██╗██║██╔══██╗  ╚██╔╝     ██║   ╚════██║",
            "██████╔╝██║  ██║██║  ██║██║██║ ╚████║██████╔╝   ██║      ██║   ███████║",
            "╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝    ╚═╝      ╚═╝   ╚══════╝"
        ]
        max_title_len = max(len(line) for line in title_lines)
        
        subtitle = "Robotics  Manager  |  Script Organization & LLMs  |  Windows/Linux"
        line_char = "─"
        decor_len = min(max_title_len, term_width - 4)
        left_margin = max(0, (term_width - max_title_len) // 2)
        
        print("\033[90m" + " " * left_margin + line_char * decor_len + "\033[0m")
        for line in title_lines:
            print("\033[91m" + " " * left_margin + line + "\033[0m")
        
        subtitle_left = max(0, (term_width - len(subtitle)) // 2)
        print(" " * subtitle_left + "\033[90m" + subtitle + "\033[0m")
        print("\033[90m" + " " * left_margin + line_char * decor_len + "\033[0m")

    # ---------- Menus navegáveis ----------
    def _menu_navegavel(self, titulo, opcoes, msg_raposa=None, subtitulo=None):
        """Exibe um menu navegável fluido, atualizando apenas as linhas necessárias."""
        # Limpa a tela e imprime o cabeçalho APENAS na primeira vez
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        if msg_raposa:
            print(fox_say(msg_raposa))
        
        term_width = shutil.get_terminal_size().columns
        menu_width = min(70, term_width - 4)
        selected = 0
        
        # Oculta o cursor do terminal para um visual mais polido
        sys.stdout.write('\033[?25l')
        sys.stdout.flush()
        
        try:
            primeira_renderizacao = True
            while True:
                # Prepara todas as linhas do menu em uma lista (buffer)
                linhas = []
                linhas.append("") # Espaço vazio antes do menu
                linhas.append("\033[90m┌" + "─" * (menu_width - 2) + "┐\033[0m")
                
                titulo_formatado = f" {titulo} ".center(menu_width - 2)
                linhas.append("\033[90m│\033[0m\033[1;96m" + titulo_formatado + "\033[0m\033[90m│\033[0m")
                
                if subtitulo:
                    subt_formatado = f" {subtitulo} ".center(menu_width - 2)
                    linhas.append("\033[90m│\033[0m" + subt_formatado + "\033[90m│\033[0m")
                
                linhas.append("\033[90m├" + "─" * (menu_width - 2) + "┤\033[0m")
                
                for i, op in enumerate(opcoes):
                    if i == selected:
                        line = f"> {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m\033[7;36m" + line + "\033[0m\033[90m│\033[0m")
                    else:
                        line = f"  {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m" + line + "\033[90m│\033[0m")
                
                linhas.append("\033[90m└" + "─" * (menu_width - 2) + "┘\033[0m")
                linhas.append("") # Espaço
                linhas.append("Use \033[93m↑/↓\033[0m para navegar, \033[92mEnter\033[0m para selecionar.")

                # Se não for a primeira vez, move o cursor para cima a quantidade exata de linhas!
                if not primeira_renderizacao:
                    sys.stdout.write(f"\033[{len(linhas)}A")
                primeira_renderizacao = False
                
                # Imprime tudo de uma vez (sem piscar a tela)
                print("\n".join(linhas))
                
                key = get_key()
                if key == 'UP':
                    selected = (selected - 1) % len(opcoes)
                elif key == 'DOWN':
                    selected = (selected + 1) % len(opcoes)
                elif key == 'ENTER':
                    return selected
                elif key == 'q':
                    return -1
        finally:
            # Garante que o cursor volte a aparecer se o menu for fechado/quebrado
            sys.stdout.write('\033[?25h')
            sys.stdout.flush()
    
    def _ler_arquivo_log(self, caminho_log):
        """Lê e retorna as últimas 20 linhas de um arquivo de log específico."""
        log_path = Path(caminho_log)
        
        if not log_path.exists():
            return f"Nenhum arquivo de log encontrado no caminho:\n'{caminho_log}'."
            
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                linhas = f.readlines()
                if not linhas:
                    return "O arquivo de log está vazio no momento."
                else:
                    # Pega as últimas 20 linhas
                    ultimas_linhas = linhas[-20:]
                    return "".join(ultimas_linhas)
        except Exception as e:
            return f"Erro ao tentar ler o arquivo de log:\n{e}"

    def _menu_logs(self):
        """Submenu para escolher entre os diferentes arquivos de log."""
        opcoes_logs = [
            "Log do Sistema",
            "Log da Simulação",
            "Voltar"
        ]
        
        while True:
            escolha = self._menu_navegavel(
                "VISUALIZADOR DE LOGS",
                opcoes_logs,
                msg_raposa="Qual arquivo de log você deseja analisar?",
                subtitulo="Selecione a origem dos logs"
            )
            
            if escolha == -1 or escolha == 2:  # Voltar ou pressionou 'q'
                break
            elif escolha == 0:
                # Log principal do sistema
                conteudo = self._ler_arquivo_log(LOG_BRAIN_FILE)
                self._exibir_texto_com_raposa("Log do Sistema (main.log)", conteudo)
            elif escolha == 1:
                # Log da simulação (ATENÇÃO: Ajuste o nome do arquivo se o seu for diferente)
                conteudo = self._ler_arquivo_log(LOG_APP_FILE)
                self._exibir_texto_com_raposa("Log da Simulação", conteudo)

    def _menu_configuracoes(self):
        """Submenu de configurações com checkboxes sem flick na tela."""
        opcoes = [
            f"Comandos por CLI      [{'x' if self.config['cli_commands'] else ' '}]",
            f"Conectar com ROS      [{'x' if self.config['ros_connection'] else ' '}]",
            f"Conectar com UDP      [{'x' if self.config['udp_connection'] else ' '}]",
            "Voltar"
        ]
        
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(fox_say("Configurações do sistema. Use ESPAÇO para alternar checkboxes.", width=60))
        
        term_width = shutil.get_terminal_size().columns
        menu_width = min(70, term_width - 4)
        selected = 0
        
        sys.stdout.write('\033[?25l')
        sys.stdout.flush()
        
        try:
            primeira_vez = True
            while True:
                linhas = []
                linhas.append("")
                linhas.append("\033[90m┌" + "─" * (menu_width - 2) + "┐\033[0m")
                titulo = " CONFIGURAÇÕES ".center(menu_width - 2)
                linhas.append("\033[90m│\033[0m\033[1;96m" + titulo + "\033[0m\033[90m│\033[0m")
                linhas.append("\033[90m├" + "─" * (menu_width - 2) + "┤\033[0m")
                
                for i, op in enumerate(opcoes):
                    if i == selected:
                        line = f"> {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m\033[7;36m" + line + "\033[0m\033[90m│\033[0m")
                    else:
                        line = f"  {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m" + line + "\033[90m│\033[0m")
                
                linhas.append("\033[90m└" + "─" * (menu_width - 2) + "┘\033[0m")
                linhas.append("")
                linhas.append("\033[90m↑/↓ Navegar   ESPAÇO Alternar   ENTER Selecionar\033[0m")
                
                if not primeira_vez:
                    sys.stdout.write(f"\033[{len(linhas)}A")
                primeira_vez = False
                
                print("\n".join(linhas))
                
                key = get_key()
                if key == 'UP':
                    selected = (selected - 1) % len(opcoes)
                elif key == 'DOWN':
                    selected = (selected + 1) % len(opcoes)
                elif key == 'SPACE':
                    if selected == 0:
                        self.config['cli_commands'] = not self.config['cli_commands']
                    elif selected == 1:
                        self.config['ros_connection'] = not self.config['ros_connection']
                    elif selected == 2:
                        self.config['udp_connection'] = not self.config['udp_connection']
                        
                    # Atualiza as opções
                    opcoes[0] = f"Comandos por CLI      [{'x' if self.config['cli_commands'] else ' '}]"
                    opcoes[1] = f"Conectar com ROS      [{'x' if self.config['ros_connection'] else ' '}]"
                    opcoes[2] = f"Conectar com UDP      [{'x' if self.config['udp_connection'] else ' '}]"
                elif key == 'ENTER':
                    if selected == 3:  # Voltar
                        break
                elif key == 'q':
                    break
        finally:
            sys.stdout.write('\033[?25h')
            sys.stdout.flush()

    def _exibir_texto_com_raposa(self, titulo, conteudo):
        """Exibe uma tela informativa com a raposa e aguarda tecla."""
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(fox_say(titulo))
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print(conteudo)
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print("\nPressione qualquer tecla para voltar...")
        get_key()  # aguarda

    # ---------- Funcionalidades originais ----------
    def _list_examples(self):
        if not self.examples_folder.exists() or not self.examples_folder.is_dir():
            self.logger.warning("A pasta 'examples' não foi encontrada.")
            return []
        examples = []
        for file in self.examples_folder.glob("*.py"):
            name = file.stem
            if name != "__init__" and not name.startswith("."):
                examples.append(name)
        return sorted(examples)

    def _choose_example(self):
        """Menu de seleção de exemplos (navegável)"""
        self.examples_list = self._list_examples()
        if not self.examples_list:
            fox_print("Nenhum exemplo encontrado na pasta 'examples/'.", width=40)
            get_key()
            return
        
        opcoes = self.examples_list + ["Voltar"]
        idx = self._menu_navegavel(
            "INICIAR SIMULAÇÃO",
            opcoes,
            msg_raposa="Escolha um exemplo para executar.",
            subtitulo=f"{len(self.examples_list)} exemplos disponíveis"
        )
        if idx is None or idx == -1 or idx == len(self.examples_list):
            return
        selected = self.examples_list[idx]
        self.logger.info(f"Iniciando exemplo: {selected}")
        try:
            module = importlib.import_module(f"examples.{selected}")

            # --- MODIFICAÇÃO AQUI ---
            # Limpa a tela para apagar o menu anterior
            os.system('cls' if os.name == 'nt' else 'clear')
            # Exibe o banner no topo
            self.banner()

            if hasattr(module, 'app'):
                fox_print(f"O exemplo '{selected}' foi iniciado. Para pausar ou cancelar clique em 'ctrl+c' ou 's'. ", width=40)
                try:
                    module.app()
                except Exception as e:
                    print("\n" + "="*50)
                    print("💥 ERRO FATAL CAPTURADO:")
                    traceback.print_exc()
                    print("="*50 + "\n")
                    input("Pressione ENTER para sair...") # Pausa a tela para você conseguir ler
            else:
                fox_print(f"O exemplo '{selected}' não tem função 'app()'.", width=40)
                get_key()
        except Exception as e:
            fox_print(f"Erro: {type(e).__name__}: {e}", width=50)
            get_key()

    def _generate_tree(self, directory: Path, prefix: str = "", max_depth: int = 3, current_depth: int = 0) -> str:
        """Gera uma representação em árvore do diretório especificado."""
        if current_depth >= max_depth:
            return ""
        
        lines = []
        try:
            items = sorted(directory.iterdir(), key=lambda p: (p.is_file(), p.name))
        except PermissionError:
            return f"{prefix}[Permissão negada]\n"
        
        # Filtra itens que não queremos mostrar
        ignore_patterns = {'__pycache__', '.git', '.venv', 'venv', 'env', '.idea', '.vscode', 'node_modules', 'build', 'dist'}
        filtered_items = [item for item in items if item.name not in ignore_patterns and not item.name.startswith('.')]
        
        for i, item in enumerate(filtered_items):
            is_last = i == len(filtered_items) - 1
            connector = "└── " if is_last else "├── "
            lines.append(f"{prefix}{connector}{item.name}")
            
            if item.is_dir():
                extension = "    " if is_last else "│   "
                subtree = self._generate_tree(item, prefix + extension, max_depth, current_depth + 1)
                if subtree:
                    lines.append(subtree.rstrip('\n'))
        
        return '\n'.join(lines)

    # ---------- Loop principal ----------
    def run(self):
        """Método principal: exibe menu principal e despacha ações."""
        intro_text = (
            "Bem-vindo ao BRAINBYTE! Eu sou a raposa guia. "
            "Aqui você gerencia infraestrutura de robótica, organiza scripts e integra LLMs. "
            "Use as setas para navegar e Enter para selecionar."
        )
        while True:
            # Removidos os clear e banners daqui, pois o próprio _menu_navegavel cuidará disso 
            # de forma inteligente ao montar a tela, evitando piscar ao voltar das opções.
            
            opcoes_principal = [
                "Iniciar simulação",
                "Ajuda",
                "Comandos",
                "Estrutura das pastas",
                "Sobre o sistema",
                "Configurações",
                "Ver Logs",
                "Sair"
            ]
            escolha = self._menu_navegavel(
                "MENU PRINCIPAL",
                opcoes_principal,
                msg_raposa=intro_text,
                subtitulo=None
            )
            
            if escolha == -1 or escolha == 7:  # Sair
                # Limpa a tela para a despedida ficar limpa
                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                print(fox_say("Até logo! Foi bom ajudar você.", width=40))
                break
            elif escolha == 0:
                self._choose_example()
            elif escolha == 1:
                self._exibir_texto_com_raposa(
                    "Ajuda",
                    "Aqui você encontra ajuda sobre as funcionalidades.\n"
                    "- Iniciar simulação: execute exemplos pré-programados.\n"
                    "- Comandos: lista de comandos disponíveis.\n"
                    "- Estrutura: mostra como o projeto está organizado.\n"
                    "- Configurações: ajuste opções do sistema.\n"
                    "- Ver Logs: exibe os últimos registros de execução."
                )
            elif escolha == 2:
                self._exibir_texto_com_raposa(
                    "Comandos Disponíveis",
                    "COMANDOS (a implementar):\n"
                    "  > run <script>  - executa um script\n"
                    "  > llm <prompt>   - consulta um modelo de linguagem\n"
                    "  > status         - mostra estado da infraestrutura"
                )
            elif escolha == 3:
                root_path = Path.cwd()
                tree_str = f"{root_path.name}\n" + self._generate_tree(root_path, max_depth=3)
                self._exibir_texto_com_raposa("Estrutura do Projeto (Profundidade 3)", tree_str)
            elif escolha == 4:
                self._exibir_texto_com_raposa(
                    "Sobre o BRAINBYTE",
                    "BRAINBYTE - Gerenciador de Infraestrutura de Robótica\n\n"
                    "Funcionalidades:\n"
                    "• Organização de scripts de simulação\n"
                    "• Interface CLI amigável com mascote raposa\n"
                    "• Configurações flexíveis (CLI/ROS)\n"
                    "• Estrutura modular pronta para expansão"
                )
            elif escolha == 5:
                self._menu_configuracoes()
            elif escolha == 6:  # <-- CHAMADA PARA OS LOGS
                self._menu_logs()