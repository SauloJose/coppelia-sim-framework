import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap
import json
from brainbyte.gui.auxF import * 

from brainbyte.utils.logging import *  
from brainbyte.core.paths import *
import traceback

import platform
import subprocess
from pathlib import Path 
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from textwrap import dedent

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
        # Carrega o arquivo de configuração de projetos
        self.pconfig_path = Path.cwd() / "brainbyte" / "utils" / "pconfig.json"
        self.pconfig = self._load_pconfig()

    def _load_description(self, description_path):
        """Carrega a descrição de um arquivo description.txt."""
        try:
            if Path(description_path).exists():
                with open(description_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    if content:
                        return content
        except Exception as e:
            self.logger.warning(f"Erro ao carregar descrição {description_path}: {e}")
        return ""

    def _get_topic_description(self, topic_name):
        """Retorna a descrição de um tópico do arquivo description.txt na pasta do tópico."""
        desc_path = Path.cwd() / "projects" / topic_name / "description.txt"
        return self._load_description(desc_path)

    def _get_project_description(self, topic_name, project_name):
        """Retorna a descrição de um projeto do arquivo description.txt na pasta do projeto."""
        desc_path = Path.cwd() / "projects" / topic_name / project_name / "description.txt"
        return self._load_description(desc_path)

    def _save_topic_description(self, topic_name, description):
        """Salva a descrição de um tópico no arquivo description.txt."""
        try:
            desc_path = Path.cwd() / "projects" / topic_name / "description.txt"
            desc_path.parent.mkdir(parents=True, exist_ok=True)
            with open(desc_path, 'w', encoding='utf-8') as f:
                f.write(description)
        except Exception as e:
            self.logger.error(f"Erro ao salvar descrição de tópico: {e}")

    def _save_project_description(self, topic_name, project_name, description):
        """Salva a descrição de um projeto no arquivo description.txt."""
        try:
            desc_path = Path.cwd() / "projects" / topic_name / project_name / "description.txt"
            desc_path.parent.mkdir(parents=True, exist_ok=True)
            with open(desc_path, 'w', encoding='utf-8') as f:
                f.write(description)
        except Exception as e:
            self.logger.error(f"Erro ao salvar descrição de projeto: {e}")

    def _load_pconfig(self):
        """Mantém compatibilidade - carrega arquivo vazio."""
        return {"topicos": {}, "projetos": {}}


    @staticmethod
    def banner():
        """Exibe o banner ASCII do BRAINBYTE com alinhamento consistente."""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        term_width = shutil.get_terminal_size().columns
        
        title_lines = [
            "██████╗ ██████╗  █████╗ ██╗███╗   ██╗██████╗ ██╗   ██╗████████╗███████╗",
            "██╔══██╗██╔══██╗██╔══██╗██║████╗  ██║██╔══██╗╚██╗ ██╔╝╚══██╔══╝██╔════╝",
            "██████╔╝██████╔╝███████║██║██╔██╗ ██║██████╔╝ ╚████╔╝    ██║   █████╗  ",
            "██╔══██╗██╔══██╗██╔══██║██║██║╚██╗██║██╔══██╗  ╚██╔╝     ██║   ██╔══╝  ",
            "██████╔╝██║  ██║██║  ██║██║██║ ╚████║██████╔╝   ██║      ██║   ███████╗",
            "╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝    ╚═╝      ╚═╝   ╚══════╝",
            "Por: Saulo José",
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
    def _menu_navegavel(self, titulo, opcoes, msg_bot=None, subtitulo=None):
        """Exibe um menu navegável fluido, atualizando apenas as linhas necessárias."""
        # Limpa a tela e imprime o cabeçalho APENAS na primeira vez
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        if msg_bot:
            print(BOT_say(msg_bot))
        
        term_width = shutil.get_terminal_size().columns
        menu_width = min(70, term_width - 4)
        selected = 0
        
        # Oculta o cursor do terminal para um visual mais polido
        sys.stdout.write('\033[?25l')
        sys.stdout.flush()
        
        try:
            primeira_renderizacao = True
            while True: #Fica num loop de criar e injetar na tela. A variável selected é utilizada até finalizar.
                # Prepara todas as linhas do menu em uma lista (buffer)
                linhas = []
                linhas.append("") # Espaço vazio antes do menu
                linhas.append("\033[90m┌" + "─" * (menu_width - 2) + "┐\033[0m") #Abre a caixa com caracteres especiais
                
                titulo_formatado = f" {titulo} ".center(menu_width - 2)
                linhas.append("\033[90m│\033[0m\033[1;96m" + titulo_formatado + "\033[0m\033[90m│\033[0m") #Espaço para título
                
                if subtitulo: #Espaço para subtítulo
                    subt_formatado = f" {subtitulo} ".center(menu_width - 2)
                    linhas.append("\033[90m│\033[0m" + subt_formatado + "\033[90m│\033[0m")
                
                linhas.append("\033[90m├" + "─" * (menu_width - 2) + "┤\033[0m") #Feixando "parágrafo " da caixa
                
                for i, op in enumerate(opcoes):
                    if i == selected:
                        line = f"> {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m\033[7;36m" + line + "\033[0m\033[90m│\033[0m") #Colore apenas a opção!
                    else: #Caso contrário só exibe
                        line = f"  {op}".ljust(menu_width - 2)
                        linhas.append("\033[90m│\033[0m" + line + "\033[90m│\033[0m")
                
                linhas.append("\033[90m└" + "─" * (menu_width - 2) + "┘\033[0m") #Aqui é só a caixa fechada
                linhas.append("") # Espaço
                linhas.append("Use \033[93m↑/↓\033[0m para navegar, \033[92mEnter\033[0m para selecionar.") #Texto informativo

                # Se não for a primeira vez, move o cursor para cima a quantidade exata de linhas!
                if not primeira_renderizacao: #Fica encima da primeira opção, mas de todo jeito estouo exibindo tudo
                    sys.stdout.write(f"\033[{len(linhas)}A")
                primeira_renderizacao = False
                
                # Imprime tudo de uma vez (sem piscar a tela)
                print("\n".join(linhas))
                
                key = get_key()
                if key == 'UP':
                    selected = (selected - 1) % len(opcoes)
                elif key == 'DOWN':
                    selected = (selected + 1) % len(opcoes)
                elif key == 'ENTER': #Retorna a opção selecionada, para ser utilizada por outra função, ou na função.
                    return selected
                elif key == 'q':
                    return -1
        finally:
            # Garante que o cursor volte a aparecer se o menu for fechado/quebrado
            sys.stdout.write('\033[?25h')
            sys.stdout.flush()
    
    def _menu_navegavel_com_descricao(self, titulo, opcoes, get_description_func, msg_bot=None, subtitulo=None):
        """Menu navegável que exibe descrição dinâmica na fala do bot conforme navega."""
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        
        term_width = shutil.get_terminal_size().columns
        menu_width = min(70, term_width - 4)
        selected = 0
        
        sys.stdout.write('\033[?25l')
        sys.stdout.flush()
        
        try:
            primeira_renderizacao = True
            linhas_anteriores = 0  # Rastrea a quantidade de linhas para apagar o console corretamente
            
            while True:
                linhas = []
                
                # --- 1. LÓGICA DO BOT (Atualizada dinamicamente) ---
                opcao_atual = opcoes[selected]
                texto_dinamico_bot = msg_bot if msg_bot else ""
                
                # Verifica se a opção atual é de voltar para suprimir a descrição
                if "voltar" in opcao_atual.lower() or opcao_atual.strip() == "..":
                    description = None
                    is_voltar = True
                else:
                    description = get_description_func(opcao_atual)
                    is_voltar = False
                
                # Adiciona a descrição à fala do bot baseado na seleção
                if description:
                    texto_dinamico_bot += f"\n\nDescrição: {description}  "
                elif not is_voltar:
                    texto_dinamico_bot += f"\n\nDescrição: (Sem descrição disponível)"
                
                # Gera a string do bot e adiciona ao topo da nossa tela
                bot_string = BOT_say(texto_dinamico_bot)
                linhas.extend(bot_string.split('\n'))
                
                # --- 2. MONTAGEM DO MENU ---
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

                # --- 3. LÓGICA DE ATUALIZAÇÃO DA TELA (Anti-flicker / Anti-sujeira) ---
                
                # Se a interface encolheu (ex: ao passar por cima de 'Voltar'), preenchemos com
                # linhas vazias para sobrescrever totalmente o rastro da tela antiga
                while len(linhas) < linhas_anteriores:
                    linhas.append("")

                # Retorna o cursor para o topo (baseado na quantidade de linhas anterior)
                if not primeira_renderizacao:
                    sys.stdout.write(f"\033[{linhas_anteriores}A")
                
                primeira_renderizacao = False
                linhas_anteriores = len(linhas)
                
                # Imprime as linhas usando \033[K para limpar resíduos horizontais da linha anterior
                for linha in linhas:
                    print(linha + "\033[K")
                
                # --- 4. CONTROLES ---
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
            sys.stdout.write('\033[?25h')
            sys.stdout.flush()

    def _ler_arquivo_log(self, caminho_log):
        """Lê e retorna as últimas 30 linhas de um arquivo de log específico."""
        log_path = Path(caminho_log)
        
        if not log_path.exists():
            return f"Nenhum arquivo de log encontrado no caminho:\n'{caminho_log}'."
            
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                linhas = f.readlines()
                if not linhas:
                    return "O arquivo de log está vazio no momento."
                else:
                    # Pega as últimas 30 linhas
                    ultimas_linhas = linhas[-30:]
                    return "".join(ultimas_linhas)
        except Exception as e:
            return f"Erro ao tentar ler o arquivo de log:\n{e}"

    def _menu_logs(self):
        """Submenu para escolher entre os diferentes arquivos de log."""
        opcoes_logs = [
            "Log do Sistema",
            "Log da Simulação",
            "Limpar todos os logs",
            "Voltar"
        ]
        
        while True:
            escolha = self._menu_navegavel(
                "VISUALIZADOR DE LOGS",
                opcoes_logs,
                msg_bot="Qual arquivo de log você deseja analisar?",
                subtitulo="Selecione a origem dos logs"
            )
            
            if escolha == -1 or escolha == 3:  # Voltar ou pressionou 'q'
                break
            elif escolha == 0:
                # Log principal do sistema
                conteudo = self._ler_arquivo_log(LOG_BRAIN_FILE)
                self._exibir_texto_com_bot("Log do Sistema (main.log)", conteudo)
            elif escolha == 1:
                # Log da simulação
                conteudo = self._ler_arquivo_log(LOG_APP_FILE)
                self._exibir_texto_com_bot("Log da Simulação", conteudo)
            elif escolha == 2:
                # Menu de confirmação (Y/N) usando a própria interface do sistema
                opcoes_confirmacao = ["Sim (Y) - Apagar tudo", "Não (N) - Cancelar"]
                confirmacao = self._menu_navegavel( #Crio outro menu navegável para escolher a ação.
                    "CONFIRMAÇÃO",
                    opcoes_confirmacao,
                    msg_bot="Deseja mesmo apagar todos os logs?\nEsta ação não poderá ser desfeita.",
                    subtitulo="Atenção!"
                )
                
                if confirmacao == 0:  # Escolheu Sim
                    try:
                        for log_file in [LOG_BRAIN_FILE, LOG_APP_FILE]:
                            log_path = Path(log_file)
                            if log_path.exists():
                                # Abrir em modo 'w' apaga o conteúdo do arquivo
                                with open(log_path, 'w', encoding='utf-8') as f:
                                    pass 
                        self._exibir_texto_com_bot(
                            "Logs Limpos", 
                            "Todos os registros de log foram apagados com sucesso!"
                        )
                    except Exception as e:
                        self._exibir_texto_com_bot(
                            "Erro ao Limpar Logs", 
                            f"Não foi possível limpar os arquivos:\n{e}"
                        )
                # Se escolher 1 (Não) ou -1 (voltar), o if é ignorado e ele simplesmente volta ao menu de logs.

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
        print(BOT_say("Configurações do sistema. Use ESPAÇO para alternar checkboxes.", width=60))
        
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

    # ---------- Funcionalidades originais ----------
    def _exibir_texto_com_bot(self, titulo, conteudo): #Padrão de desenho na tela!
        """Exibe uma tela informativa com a bot e aguarda tecla."""
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say(titulo))
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print(conteudo)
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print("\nPressione qualquer tecla para voltar...")

        self._flush_input() #limpa eventuais sobras
        get_key()  # aguarda

        sys.stdout.write('\033[?25h')
        sys.stdout.flush()

    def _flush_input(self):
        """Esvazia o buffer de entrada, descartando teclas pendentes."""
        if os.name == 'nt':
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getch()
        else:
            import select
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while True:
                    r, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if r:
                        sys.stdin.read(1)
                    else:
                        break
            except:
                pass
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _list_topics(self):
        """Lista as subpastas em 'projects/' que servem como categorias/tópicos."""
        target_dir = Path.cwd() / "projects"
        
        if not target_dir.exists() or not target_dir.is_dir():
            self.logger.warning("A pasta 'projects' não foi encontrada.")
            return []
            
        topics = []
        for item in target_dir.iterdir():
            # Critérios: É uma pasta e não é oculta
            if item.is_dir() and not item.name.startswith((".", "__")):
                topics.append(item.name)
                
        return sorted(topics) #Retorna o nome das subpastas de projects/
    
    def _list_projects_in_topic(self,topic_name):
        """Lista as subpastas em 'projects/' que contêm um script .py correspondente"""
        # Certifique-se que self.projects_folder aponta para a pasta 'projects'
        target_dir = Path.cwd() / "projects" / topic_name #Diretório de procura
        
        if not target_dir.exists() or not target_dir.is_dir():
            self.logger.warning("A pasta 'projects' não foi encontrada.")
            return []
            
        projects = []
        # Varre todos os itens dentro de projects/
        for item in target_dir.iterdir():
            # Critérios: É uma pasta? Não é oculta?
            if item.is_dir() and not item.name.startswith((".", "__")):
                # Verifica se existe o script .py com o mesmo nome da pasta lá dentro
                script_file = item / f"{item.name}.py"
                if script_file.exists():
                    projects.append(item.name)
                    
        return sorted(projects)

    def _choose_project(self):
        """Menu de seleção de projetos dentro da pasta 'projects/'"""
        # Escolher o tópico
        topics_list = self._list_topics()

        if not topics_list:
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()
            print(BOT_say("Não há nenhum tópico encontrado dentro dessa pasta."))
            get_key()
            return
        
        opcoes_topicos = topics_list + ["Voltar"]
        
        # Função para obter descrição de tópico
        def get_topic_desc(topic_name):
            if topic_name == "Voltar":
                return ""
            return self._get_topic_description(topic_name)
        
        # Menu com descrição dinâmica para tópicos
        idx_topico = self._menu_navegavel_com_descricao(
            "ESCOLHER TÓPICO",
            opcoes_topicos,
            get_topic_desc,
            msg_bot="Escolha a categoria do projeto.",
            subtitulo=f"{len(topics_list)} tópicos disponíveis"
        )

        if idx_topico is None or idx_topico == -1 or idx_topico == len(topics_list):
            return
            
        selected_topic = topics_list[idx_topico]

        # Agora chamamos o método atualizado que lista pastas
        projects_list = self._list_projects_in_topic(selected_topic)
        
        # CORREÇÃO AQUI: Se não houver projetos, limpa tudo e exibe apenas a fala do robô de forma limpa
        if not projects_list:
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()
            print(BOT_say("Não há nenhum projeto dentro dessa pasta."))
            get_key()
            return
        
        opcoes_projetos = projects_list + ["Voltar"]
        
        # Função para obter descrição de projeto
        def get_project_desc(project_name):
            if project_name == "Voltar":
                return ""
            return self._get_project_description(selected_topic, project_name)
        
        # Menu com descrição dinâmica para projetos
        idx_proj = self._menu_navegavel_com_descricao(
            f"TÓPICO: {selected_topic.upper()}",
            opcoes_projetos,
            get_project_desc,
            msg_bot="Agora, escolha o projeto para executar.",
            subtitulo=f"{len(projects_list)} projetos disponíveis"
        )

        if idx_proj is None or idx_proj == -1 or idx_proj == len(projects_list):
            return

        selected_project = projects_list[idx_proj]
        self.logger.info(f"Iniciando projeto: {selected_topic}/{selected_project}")
        
        try:
            # LÓGICA DE IMPORTAÇÃO: projects.NomeDaPasta.NomeDoArquivo
            module_path = f"projects.{selected_topic}.{selected_project}.{selected_project}"
            module = importlib.import_module(module_path)

            importlib.reload(module)
            
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()

            if hasattr(module, 'app'):
                BOT_print("Verificando conexão com o CoppeliaSim...", width=50)
                
                # Verifica se o simulador está aberto ANTES de rodar
                if not self._is_coppeliasim_running():
                    os.system('cls' if os.name == 'nt' else 'clear')
                    self.banner()
                    BOT_print("Erro: O CoppeliaSim não está rodando na porta 23000.\n"
                              "Abra o simulador e inicie a cena antes de rodar o projeto.", width=55)
                    get_key()
                    return

                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                BOT_print(f"O projeto '{selected_project}' foi iniciado com sucesso!\n"
                          f"Acompanhe a execução no terminal e os gráficos na tela.\n"
                          f"Pressione Ctrl+C para encerrar a simulação.", width=55)

                # Executa com timeout=None para rodar na Thread Principal com segurança
                status, info = self._run_module_app(module, timeout=None)

                if status == 'exception':
                    e, tb = info
                    print("\n" + "="*50)
                    print("ERRO FATAL NO PROJETO:")
                    print(tb)
                    print("="*50 + "\n")
                    input("Pressione ENTER para voltar ao menu...")
                else:
                    # Sucesso: terminou normalmente ou via Ctrl+C limpo
                    os.system('cls' if os.name == 'nt' else 'clear')
                    self.banner()
                    BOT_print("Projeto finalizado com sucesso.", width=40)
                    get_key()
            else:
                BOT_print(f"Erro: O arquivo '{selected_project}.py' não contém a função 'app()'.", width=45)
                get_key()
                
        except Exception as e:
            BOT_print(f"Erro ao carregar módulo: {type(e).__name__}: {e}", width=50)
            get_key()

    def _create_new_simulation(self):
        """Fluxo completo: escolher tópico (existente/novo) e depois criar ou copiar projeto."""

        # ── Etapa 0: limpa tela e cabeçalho ──
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say("Vamos criar uma nova simulação!", width=65))

        # Garante que a pasta projects exista
        projects_dir = Path.cwd() / "projects"
        projects_dir.mkdir(parents=True, exist_ok=True)

        # ── Etapa 1: escolher tópico ──
        opcoes_topico = ["Tópico existente", "Novo tópico", "Voltar"]
        escolha_topico = self._menu_navegavel(
            "CRIAÇÃO DE SIMULAÇÃO",
            opcoes_topico,
            msg_bot="Você quer usar um tópico já existente ou criar um novo?",
            subtitulo="Etapa 1: Tópico"
        )
        if escolha_topico == -1 or escolha_topico == 2:  # Voltar
            return

        nome_topico_limpo = ""
        desc_topico = ""

        if escolha_topico == 0:  # Tópico existente
            topics_list = self._list_topics()
            if not topics_list:
                self._exibir_texto_com_bot("Aviso", "Nenhum tópico encontrado. Crie um novo tópico primeiro.")
                return

            # Reaproveita o menu com descrição dinâmica para escolher tópico
            opcoes_existentes = topics_list + ["Voltar"]

            def get_topic_desc(topic_name):
                if topic_name == "Voltar":
                    return ""
                return self._get_topic_description(topic_name)

            idx = self._menu_navegavel_com_descricao(
                "TÓPICO EXISTENTE",
                opcoes_existentes,
                get_topic_desc,
                msg_bot="Escolha o tópico onde o projeto será salvo.",
                subtitulo=f"{len(topics_list)} tópicos disponíveis"
            )
            if idx == -1 or idx == len(topics_list):  # Voltar
                return
            nome_topico_limpo = topics_list[idx]
            desc_topico = self._get_topic_description(nome_topico_limpo)
            # Não solicita descrição, já existe

        else:  # Novo tópico (escolha_topico == 1)
            # Mostra tela "limpa" com bot para input
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()
            print(BOT_say("Criação de novo tópico", width=50))
            print("\n" + "\033[90m" + "─" * 70 + "\033[0m")

            # Reexibe tópicos existentes para referência
            topicos_existentes = [d.name for d in projects_dir.iterdir() if d.is_dir() and not d.name.startswith('__')]
            if topicos_existentes:
                print("\n\033[96mTópicos já existentes:\033[0m")
                for t in sorted(topicos_existentes):
                    print(f"  \033[90m-\033[0m {t}")
                print()

            nome_topico_raw = input("\033[92m> \033[0mNome do novo tópico (ex: locomocao): ").strip()
            if not nome_topico_raw:
                self._exibir_texto_com_bot("Aviso", "Nome do tópico vazio. Criação cancelada.")
                return
            nome_topico_limpo = nome_topico_raw.replace(' ', '_').lower()

            desc_topico = input("\033[92m> \033[0mDescrição do tópico (opcional): ").strip()
            if desc_topico:
                self._save_topic_description(nome_topico_limpo, desc_topico)

        # ── Etapa 2: escolher ação do projeto ──
        opcoes_projeto = ["Novo projeto", "Copiar Projeto existente", "Voltar"]
        escolha_proj = self._menu_navegavel(
            "CRIAÇÃO DE PROJETO",
            opcoes_projeto,
            msg_bot=f"Tópico: {nome_topico_limpo}\nComo deseja criar o projeto?",
            subtitulo="Etapa 2: Projeto"
        )
        if escolha_proj == -1 or escolha_proj == 2:  # Voltar
            return

        if escolha_proj == 0:  # Novo projeto
            self._create_new_project_scratch(nome_topico_limpo)
        else:  # Copiar projeto existente
            self._copy_existing_project(nome_topico_limpo)

    # ---------- Funcionalidade para navegar no projeto ---------
    def _create_new_project_scratch(self, nome_topico_limpo):
        """Cria um novo projeto a partir dos templates (fluxo original)."""

        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say("Novo projeto - Preencha os dados abaixo", width=65))
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")

        sys.stdout.write('\033[?25h')
        sys.stdout.flush()

        try:
            # Coleta os dados
            nome_aplicacao = input("\033[92m> \033[0mNome da aplicação (ex: MeuRobo): ").strip()
            if not nome_aplicacao:
                self._exibir_texto_com_bot("Aviso", "Nome da aplicação vazio. Criação cancelada.")
                return

            tempo_simulacao = input("\033[92m> \033[0mTempo de simulação (em segundos): ").strip()
            nome_cena = input("\033[92m> \033[0mNome da cena (ex: cena_basica): ").strip()

            print("\n\033[96m--- Descrição (opcional) ---\033[0m")
            desc_projeto = input("\033[92m> \033[0mDescrição do projeto: ").strip()

            # Limpeza dos nomes
            nome_aplicacao_limpo = nome_aplicacao.replace('.py', '').replace(' ', '')
            nome_cena_limpo = nome_cena.replace('.ttt', '').replace(' ', '_')

            base_dir = Path.cwd()

            # Caminhos dos templates
            template_app = base_dir / "brainbyte" / "utils" / "basics" / "app.txt"
            template_scene = base_dir / "brainbyte" / "utils" / "basics" / "scene.ttt"

            # Pasta destino
            sim_folder = base_dir / "projects" / nome_topico_limpo / nome_aplicacao_limpo
            sim_folder.mkdir(parents=True, exist_ok=True)

            arquivo_py = f"{nome_aplicacao_limpo}.py"
            arquivo_ttt = f"{nome_cena_limpo}.ttt"

            caminho_novo_app = sim_folder / arquivo_py
            caminho_nova_cena = sim_folder / arquivo_ttt

            # Gera .py a partir do template
            if not template_app.exists():
                raise FileNotFoundError(f"Template de app não encontrado: {template_app}")
            with open(template_app, 'r', encoding='utf-8') as f:
                conteudo_template = f.read()
            conteudo_final = conteudo_template.replace("{name_app}", nome_aplicacao_limpo)
            conteudo_final = conteudo_final.replace("{simulation_time}", tempo_simulacao)
            conteudo_final = conteudo_final.replace("{name_scene}", nome_cena_limpo)
            with open(caminho_novo_app, 'w', encoding='utf-8') as f:
                f.write(conteudo_final)

            # Copia cena
            if not template_scene.exists():
                raise FileNotFoundError(f"Template de cena não encontrado: {template_scene}")
            shutil.copy2(template_scene, caminho_nova_cena)

            # Salva descrição do projeto
            if desc_projeto:
                self._save_project_description(nome_topico_limpo, nome_aplicacao_limpo, desc_projeto)

            # Feedback de sucesso
            mensagem_sucesso = (
                f"Simulação criada com sucesso!\n\n"
                f"📁 Projeto salvo em: projects/{nome_topico_limpo}/{nome_aplicacao_limpo}/\n"
                f"📄 Script principal: {arquivo_py}\n"
                f"📄 Cena do Coppelia: {arquivo_ttt}\n\n"
                f"O arquivo criado irá abrir para edições!"
            )
            self._exibir_texto_com_bot("Sucesso!", mensagem_sucesso)

            # Abre o .py no editor
            path_py_str = str(caminho_novo_app.resolve())
            try:
                if platform.system() == 'Windows':
                    os.startfile(path_py_str)
                elif platform.system() == 'Darwin':
                    subprocess.call(('open', path_py_str))
                else:
                    subprocess.call(('xdg-open', path_py_str))
            except Exception as e:
                self.logger.warning(f"Não foi possível abrir o arquivo: {e}")

            # Carrega cena no CoppeliaSim (se aberto)
            try:
                client = RemoteAPIClient()
                sim = client.require('sim')
                path_cena_str = str(caminho_nova_cena.resolve())
                sim.loadScene(path_cena_str)
                self.logger.info(f"Cena {arquivo_ttt} carregada com sucesso.")
            except Exception as e:
                self.logger.warning(f"CoppeliaSim não disponível para carregar cena: {e}")

        except Exception as e:
            msg = traceback.format_exc()
            self.logger.error(f"Erro ao criar projeto: {msg}")
            self._exibir_texto_com_bot("Erro", f"Não foi possível criar o projeto:\n{e}")
        finally:
            sys.stdout.write('\033[?25l')
            sys.stdout.flush()

    def _run_module_app(self, module, timeout=None):
        """
        Executa module.app(). Se timeout for None, roda diretamente na
        Thread Principal para garantir compatibilidade com Matplotlib e Ctrl+C.
        """
        import traceback

        # Se não houver timeout, roda na Thread Principal (Nativo e Seguro)
        if timeout is None:
            try:
                module.app()
                return 'success', None
            except KeyboardInterrupt:
                # Captura o Ctrl+C caso o próprio app não o trate completamente
                return 'success', None
            except Exception as e:
                return 'exception', (e, traceback.format_exc())

        # Caso precise de timeout (Lógica antiga em thread secundária)
        import threading
        exception_info = None
        completed = threading.Event()

        def target():
            nonlocal exception_info
            try:
                module.app()
            except Exception as e:
                exception_info = (e, traceback.format_exc())
            finally:
                completed.set()

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()
        finished = completed.wait(timeout=timeout)

        if not finished:
            return 'timeout', None
        if exception_info:
            return 'exception', exception_info
        return 'success', None

    def _is_coppeliasim_running(self, host='127.0.0.1', port=23000, timeout=0.5):
        """Retorna True se o CoppeliaSim estiver ouvindo na porta padrão."""
        import socket
        try:
            s = socket.create_connection((host, port), timeout=timeout)
            s.close()
            return True
        except:
            return False
    
    def _copy_existing_project(self, nome_topico_destino):
        """Copia um projeto existente para dentro do tópico escolhido, com novo nome."""

        # 1. Selecionar projeto de origem
        source_topic, source_project = self._select_project_interactively()
        if not source_topic or not source_project:
            return  # Usuário cancelou

        # 2. Pedir novo nome para o projeto
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say("Copiar projeto existente", width=60))
        print(f"\n\033[90mOrigem: projects/{source_topic}/{source_project}\033[0m")
        print(f"\033[90mDestino: projects/{nome_topico_destino}/<novo_nome>\033[0m\n")

        sys.stdout.write('\033[?25h')
        sys.stdout.flush()

        novo_nome = input("\033[92m> \033[0mNovo nome para o projeto copiado: ").strip()
        if not novo_nome:
            self._exibir_texto_com_bot("Aviso", "Nome vazio. Cópia cancelada.")
            return

        novo_nome_limpo = novo_nome.replace(' ', '').replace('.py', '')
        desc_projeto = input("\033[92m> \033[0mNova descrição (opcional, Enter mantém original): ").strip()

        # Se descrição em branco, podemos tentar carregar a original do projeto fonte
        if not desc_projeto:
            desc_original = self._get_project_description(source_topic, source_project)
        else:
            desc_original = desc_projeto

        base_dir = Path.cwd()
        origem_path = base_dir / "projects" / source_topic / source_project
        destino_path = base_dir / "projects" / nome_topico_destino / novo_nome_limpo

        if not origem_path.exists():
            self._exibir_texto_com_bot("Erro", f"Projeto de origem não encontrado:\n{origem_path}")
            return

        try:
            # Copia toda a pasta do projeto
            shutil.copytree(origem_path, destino_path)
            # Se houver um arquivo .py com o nome antigo, renomeia para o novo nome
            old_py = destino_path / f"{source_project}.py"
            new_py = destino_path / f"{novo_nome_limpo}.py"
            if old_py.exists() and old_py != new_py:
                old_py.rename(new_py)
            # Atualiza a descrição se foi fornecida
            if desc_original:
                self._save_project_description(nome_topico_destino, novo_nome_limpo, desc_original)

            mensagem = (
                f"Projeto copiado com sucesso!\n\n"
                f"📍 Novo projeto: projects/{nome_topico_destino}/{novo_nome_limpo}/\n"
                f"📄 Script principal: {novo_nome_limpo}.py"
            )
            self._exibir_texto_com_bot("Cópia concluída", mensagem)
        except Exception as e:
            msg = traceback.format_exc()
            self.logger.error(f"Erro ao copiar projeto: {msg}")
            self._exibir_texto_com_bot("Erro", f"Falha ao copiar projeto:\n{e}")
        finally:
            sys.stdout.write('\033[?25l')
            sys.stdout.flush()

    def _select_project_interactively(self):
        """Usa os menus para que o usuário escolha um tópico e um projeto.
        Retorna (topic_name, project_name) ou (None, None) se cancelar."""

        topics_list = self._list_topics()
        if not topics_list:
            self._exibir_texto_com_bot("Aviso", "Nenhum tópico disponível para copiar.")
            return None, None

        opcoes_topicos = topics_list + ["Voltar"]

        def get_topic_desc(topic_name):
            if topic_name == "Voltar":
                return ""
            return self._get_topic_description(topic_name)

        idx_topico = self._menu_navegavel_com_descricao(
            "SELECIONE O TÓPICO DE ORIGEM",
            opcoes_topicos,
            get_topic_desc,
            msg_bot="De qual tópico você deseja copiar o projeto?",
            subtitulo=f"{len(topics_list)} tópicos disponíveis"
        )
        if idx_topico == -1 or idx_topico == len(topics_list):
            return None, None

        selected_topic = topics_list[idx_topico]

        projects_list = self._list_projects_in_topic(selected_topic)
        if not projects_list:
            self._exibir_texto_com_bot("Aviso", f"O tópico '{selected_topic}' não possui projetos.")
            return None, None

        opcoes_projetos = projects_list + ["Voltar"]

        def get_project_desc(project_name):
            if project_name == "Voltar":
                return ""
            return self._get_project_description(selected_topic, project_name)

        idx_proj = self._menu_navegavel_com_descricao(
            f"PROJETOS EM {selected_topic.upper()}",
            opcoes_projetos,
            get_project_desc,
            msg_bot="Escolha o projeto que será copiado.",
            subtitulo=f"{len(projects_list)} projetos disponíveis"
        )
        if idx_proj == -1 or idx_proj == len(projects_list):
            return None, None

        return selected_topic, projects_list[idx_proj]

    def _navegate_project(self):
        """
        Navegador interativo de arquivos estilo terminal.
        Exibe a árvore com profundidade 1 e aceita comandos.
        """

        # Ponto de partida virtual
        self.current_nav_path = Path.cwd().resolve()

        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say("Navegador de projeto. Digite 'help' para ver comandos."))
        current_depth = 1 
        erro_cmd = False 
        # Loop principal do navegador
        while True:
            # Exibe caminho atual e árvore
            print(f"\n\033[1;96m📁 {self.current_nav_path}\033[0m")
            tree = self._generate_tree(self.current_nav_path, max_depth=current_depth)
            if tree.strip():
                print(tree)
            else:
                print("   (pasta vazia)")


            # Prompt de comando
            cmd_input = input("\n\033[92m> \033[0m").strip()
            if not cmd_input:
                current_depth = 1
                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                print(BOT_say("Navegador de projeto. Digite 'help' para ver comandos."))
                continue
            
            parts = cmd_input.split(maxsplit=1)
            command = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else ""
            
            if command in ("exit", "q", "quit"):
                current_depth = 1
                break
            elif command == "help":
                self._show_nav_help()
                current_depth = 1
            elif command in ("ls", "dir", "tree"):
                if arg.isdigit():
                    current_depth = int(arg)
                else:
                    current_depth = 3
            elif command == "cd":
                self._nav_change_directory(arg)
                current_depth = 1
            elif command == "open":
                self._nav_open_file(arg)
                current_depth = 1
            elif command == "del":
                self._nav_del_file(arg)
                current_depth = 1
            else:
                erro_cmd = True 
                current_depth = 1
            
            # Limpa a tela para próxima iteração (opcional, para manter a navegação limpa)
            # Se quiser manter histórico, remova os clears abaixo.
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()
            if erro_cmd:
                print(BOT_say(f"Comando desconhecido: '{command}'. Digite 'help'.", width=50))
                current_depth = 1
            else:
                print(BOT_say("Navegador de projeto. Digite 'help' para ver comandos."))  

            #reseto erro
            erro_cmd = False 

    def _show_nav_help(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        # O dedent remove os espaços à esquerda gerados pela indentação do código
        help_text = dedent("""\
            Comandos disponíveis:

            cd <pasta>     - Entra na pasta especificada
            open <arquivo> - Abre o arquivo no editor padrão
            ls / tree      - Reexibe a estrutura da pasta atual
            help           - Mostra esta ajuda
            exit / q       - Sai do navegador e volta ao menu
            del            - Deleta um arquivo
        """).strip() # .strip() remove a quebra de linha extra do começo/fim

        print(BOT_say(help_text, width=60))
        input("\nPressione ENTER para continuar...")
    
    def _nav_change_directory(self, arg):
        if not arg:
            print("Uso: cd <pasta>")
            input("Pressione ENTER...")
            return
        
        # Trata caminhos absolutos ou relativos
        new_path = (self.current_nav_path / arg).resolve()
        
        if not new_path.exists():
            print(f"A pasta '{arg}' não existe.")
            input("Pressione ENTER...")
            return
        if not new_path.is_dir():
            print(f"'{arg}' não é uma pasta.")
            input("Pressione ENTER...")
            return
        
        # Verifica permissão de leitura
        if not os.access(new_path, os.R_OK):
            print(f"Sem permissão para acessar '{arg}'.")
            input("Pressione ENTER...")
            return
        
        self.current_nav_path = new_path

    def _nav_del_file(self, arg):
        """ Definindo função para apagar o arquivo ou pasta de projeto"""
        import shutil  # Recomendo colocar esse import lá no topo do seu script

        if not arg:
            print("Uso: del <arquivo_ou_pasta>")
            input("Pressione ENTER...")
            return
        
        target_path = (self.current_nav_path / arg).resolve()

        if not target_path.exists():
            print(f"Arquivo ou pasta '{arg}' não encontrado.")
            input("Pressione ENTER...")
            return
        
        # Tela de confirmação
        print(f"\n⚠️  AVISO: Você está prestes a apagar permanentemente '{arg}'.")
        if target_path.is_dir():
            print("Isso apagará a pasta inteira e TODOS os arquivos da simulação dentro dela!")
        
        confirmacao = input("Tem certeza absoluta que deseja continuar? (s/n): ").strip().lower()

        if confirmacao != 's':
            print("Operação de exclusão cancelada. Ufa! 🦊")
            input("Pressione ENTER para voltar...")
            return
        
        # Executa a exclusão
        try:
            if target_path.is_file():
                target_path.unlink()  # Deleta arquivo único
                print(f"Arquivo '{target_path.name}' apagado com sucesso.")
            elif target_path.is_dir():
                shutil.rmtree(target_path)  # Deleta a pasta e tudo o que tem dentro
                print(f"Projeto '{target_path.name}' deletado com sucesso.")
            
            input("Pressione ENTER...")
        except Exception as e:
            print(f"Erro ao tentar apagar: {e}")
            input("Pressione ENTER...")

    def _nav_open_file(self, arg):
        if not arg:
            print("Uso: open <arquivo>")
            input("Pressione ENTER...")
            return
        
        file_path = (self.current_nav_path / arg).resolve()
        
        if not file_path.exists():
            print(f"Arquivo '{arg}' não encontrado.")
            input("Pressione ENTER...")
            return
        if not file_path.is_file():
            print(f"'{arg}' não é um arquivo.")
            input("Pressione ENTER...")
            return
        
        # Abre com editor padrão do sistema
        try:
            if os.name == 'nt':  # Windows
                os.startfile(file_path)
            elif os.name == 'posix':  # Linux/Mac
                import subprocess
                subprocess.run(['xdg-open', str(file_path)], check=True)
            else:
                print("Sistema operacional não suportado para abertura automática.")
                input("Pressione ENTER...")
                return
            
            print(f"Arquivo '{file_path.name}' aberto no editor externo.")
            print("⚠️  Modifique e salve o arquivo normalmente. Ao fechar o editor, você retornará ao navegador.")
            input("Pressione ENTER quando terminar...")
        except Exception as e:
            print(f"Erro ao abrir o arquivo: {e}")
            input("Pressione ENTER...")

    def _generate_tree(self, directory: Path, prefix: str = "", max_depth: int = 2, current_depth: int = 0) -> str:
        """Gera uma representação em árvore do diretório especificado."""
        if current_depth >= max_depth:
            return ""
        
        lines = []
        try:
            items = sorted(directory.iterdir(), key=lambda p: (p.is_file(), p.name))
        except PermissionError:
            return f"{prefix}[Permissão negada]\n"
        
        # Filtra itens que não queremos mostrar
        ignore_patterns = {'__init__.py','config.json','pyproject.toml','setup.py','requirements.txt','requirements-dev.txt', '__pycache__', '.git', '.venv', 'venv', 'env', '.idea', '.vscode', 'node_modules', 'build', 'dist','description.txt'}
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

    def _delete_project(self):
        """Deleta um projeto inteiro após seleção por menus e confirmação."""

        # Seleciona tópico e projeto usando o mesmo fluxo interativo
        selected_topic, selected_project = self._select_project_interactively()
        if not selected_topic or not selected_project:
            return  # Usuário cancelou a seleção

        # Caminho completo da pasta do projeto
        projeto_path = Path.cwd() / "projects" / selected_topic / selected_project

        # Confirmação via menu (Sim/Não)
        opcoes_confirmacao = [
            "Sim, deletar permanentemente",
            "Não, cancelar"
        ]
        confirm = self._menu_navegavel(
            "CONFIRMAR EXCLUSÃO",
            opcoes_confirmacao,
            msg_bot=f"Tem certeza que deseja apagar o projeto?\n\n"
                    f"📁 {selected_topic}/{selected_project}\n\n"
                    f"Esta ação não pode ser desfeita!",
            subtitulo="⚠️  Atenção!"
        )

        if confirm != 0:  # Qualquer coisa diferente de Sim (0) cancela
            self._exibir_texto_com_bot("Cancelado", "A exclusão do projeto foi cancelada.")
            return

        # Executa a exclusão
        try:
            shutil.rmtree(projeto_path)
            self.logger.info(f"Projeto deletado: {selected_topic}/{selected_project}")
            self._exibir_texto_com_bot(
                "Projeto deletado",
                f"O projeto '{selected_project}' foi removido com sucesso do tópico '{selected_topic}'."
            )
        except Exception as e:
            self.logger.error(f"Erro ao deletar projeto: {e}")
            self._exibir_texto_com_bot(
                "Erro",
                f"Não foi possível deletar o projeto:\n{e}"
            )

    def __clear_all_window(self):
        """
            Garantir que a tela esteja completamente limpa
        """ 
        if os.name == 'nt':
            os.system('cls')
        else:
            # \033[2J (limpa tela), \033[3J (limpa scrollback), \033[H (reseta cursor)
            sys.stdout.write("\033[2J\033[3J\033[H")
            sys.stdout.flush()

    # ---------- Loop principal ----------
    def run(self):
        """Método principal: exibe menu principal e despacha ações."""
        intro_text = (
            "Bem-vindo ao BRAINBYTE! Eu sou o Blue, seu agente guia. "
            "Aqui você gerencia infraestrutura de robótica, organiza scripts e integra LLMs. "
            "Use as setas para navegar e Enter para selecionar."
        )
        while True:
            self.__clear_all_window()

            opcoes_principal = [
                "Iniciar simulação",      # 0
                "Criar nova simulação",   # 1
                "Deletar projeto",        # 2  ← NOVA OPÇÃO 
                "Navegar pelo projeto",   # 3
                "Ver Logs",               # 4
                "Configurações",          # 5
                "Ajuda",                  # 6
                "Sobre o sistema",        # 7
                "Sair"                    # 8  
            ]
            escolha = self._menu_navegavel(
                "MENU PRINCIPAL",
                opcoes_principal,
                msg_bot=intro_text,
                subtitulo=None
            )
            
            if escolha == -1 or escolha == 8:  # Sair agora é índice 8
                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                print(BOT_say("Até logo! Foi bom ajudar você.", width=40))
                break
            elif escolha == 0:   # Iniciar simulação
                self._choose_project()
            elif escolha == 1:   # Criar nova simulação
                self._create_new_simulation()
            elif escolha == 2:   # Deletar projeto
                self._delete_project()
            elif escolha == 3:   # Navegar pelo projeto
                self._navegate_project()
            elif escolha == 4:   # Ver Logs
                self._menu_logs()
            elif escolha == 5:   # Configurações
                self._menu_configuracoes()
            elif escolha == 6:   # Ajuda
                self._exibir_texto_com_bot(
                    "Aqui você encontra ajuda sobre as funcionalidades.\n"
                    "- Iniciar simulação: execute exemplos pré-programados.\n"
                    "- Criar nova simulação: crie suas próprias simulações.\n"
                    "- Deletar projeto: remova um projeto permanentemente.\n"
                    "- Navegar pelo projeto: explore a estrutura de arquivos.\n"
                    "- Configurações: ajuste opções do sistema.\n"
                    "- Ver Logs: exibe os últimos registros de execução.",'Espero que tenha lhe ajudado.'
                )
            elif escolha == 7:   # Sobre o sistema
                self._exibir_texto_com_bot(
                    "BRAINBYTE - Gerenciador de Infraestrutura de Robótica\n\n"
                    "Funcionalidades:\n"
                    "• Organização de scripts de simulação\n"
                    "• Interface CLI amigável com mascote bot\n"
                    "• Configurações flexíveis (CLI/ROS)\n"
                    "• Estrutura modular pronta para expansão",'Espero que tenha lhe ajudado.'
                )