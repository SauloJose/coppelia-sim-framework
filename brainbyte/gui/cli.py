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
import msvcrt

import platform
import subprocess
from pathlib import Path # Garantindo que Path esteja disponível, caso não esteja no topo do arquivo
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

    def _exibir_texto_com_bot(self, titulo, conteudo): #Padrão de desenho na tela!
        """Exibe uma tela informativa com a bot e aguarda tecla."""
        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say(titulo))
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print(conteudo)
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        print("\nPressione qualquer tecla para voltar...")
        get_key()  # aguarda

    # ---------- Funcionalidades originais ----------
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
        #Escolher o tópico
        topics_list = self._list_topics()

        if not topics_list:
            BOT_print("Nenhum tópico encontrado na pasta 'projects/'.", width=40)
            get_key()
            return
        
        opcoes_topicos = topics_list + ["Voltar"]
        idx_topico = self._menu_navegavel( #Menu navegável para encontrar a opção de tópicos
            "ESCOLHER TÓPICO",
            opcoes_topicos,
            msg_bot="Escolha a categoria do projeto.",
            subtitulo=f"{len(topics_list)} tópicos disponíveis"
        )

        if idx_topico is None or idx_topico == -1 or idx_topico == len(topics_list): #Garante que é um idx real
            return
            
        selected_topic = topics_list[idx_topico] #Pego o tópico correto (Vem o nome dos tópicos.)

        # Agora chamamos o método atualizado que lista pastas
        projects_list = self._list_projects_in_topic(selected_topic)  #Puxo a lista de pastas dentro do diretório com o tópico escolhido.
        
        if not projects_list:
            BOT_print("Nenhum projeto encontrado na pasta 'projects/'.", width=40)
            get_key()
            return
        
        opcoes_projetos = projects_list + ["Voltar"]
        idx_proj = self._menu_navegavel( #Gero outro menu navegável para escolher o projeto
            f"TÓPICO: {selected_topic.upper()}",
            opcoes_projetos,
            msg_bot="Agora, escolha o projeto para executar.",
            subtitulo=f"{len(projects_list)} projetos disponíveis"
        )

        if idx_proj is None or idx_proj == -1 or idx_proj == len(projects_list):
            return

        selected_project = projects_list[idx_proj]
        self.logger.info(f"Iniciando projeto: {selected_topic}/{selected_project}") #O projeto que foi selecionado.
        
        try:
            # LÓGICA DE IMPORTAÇÃO: projects.NomeDaPasta.NomeDoArquivo
            # Ex: projects.MeuRobo.MeuRobo
            module_path = f"projects.{selected_topic}.{selected_project}.{selected_project}"
            module = importlib.import_module(module_path) # pego o ponteiro para o objeto do módulo que eu escolhi

            importlib.reload(module)  # reinicio o módulo, garantindo que seja a primeira execução dele
            
            os.system('cls' if os.name == 'nt' else 'clear') #Limpo a interface gráfica
            self.banner()

            if hasattr(module, 'app'): #Verifico se no módulo tem o app(), que irá o abrir.
                BOT_print(f"O projeto '{selected_project}' ({selected_topic}) foi iniciado. Para pausar ou cancelar clique em 'ctrl+c' ou 'x'.", width=45)
                try:
                    module.app()  #Executo o app() que está dentro do módulo
                except Exception as e:
                    print("\n" + "="*50)
                    print("ERRO FATAL NO PROJETO:")
                    traceback.print_exc()
                    print("="*50 + "\n")
                    input("Pressione ENTER para voltar ao menu...")
            else:
                BOT_print(f"Erro: O arquivo '{selected_project}.py' não contém a função 'app()'.", width=45)
                get_key()

            msvcrt.getch()
                
        except Exception as e:
            BOT_print(f"Erro ao carregar módulo: {type(e).__name__}: {e}", width=50)
            get_key()

            
    def _create_new_simulation(self):
        """Coleta dados do usuário e gera a pasta do projeto com os scripts e cena."""

        os.system('cls' if os.name == 'nt' else 'clear')
        self.banner()
        print(BOT_say("Vamos criar uma nova simulação! Preencha os dados abaixo.", width=65))
        print("\n" + "\033[90m" + "─" * 70 + "\033[0m")
        
        # Ocultar o cursor no menu navegável é ótimo, mas aqui precisamos dele para o usuário digitar!
        sys.stdout.write('\033[?25h')
        sys.stdout.flush()

        try:
            base_dir = Path.cwd()
            projects_dir = base_dir / "projects"
            
            # Garante que a pasta projects exista para podermos listar
            projects_dir.mkdir(parents=True, exist_ok=True)
            
            # Verifica e exibe os tópicos existentes
            topicos_existentes = [d.name for d in projects_dir.iterdir() if d.is_dir() and not d.name.startswith('__')]
            
            if topicos_existentes:
                print("\n\033[96mTópicos já existentes:\033[0m")
                for t in sorted(topicos_existentes):
                    print(f"  \033[90m-\033[0m {t}")
                print("\033[90m(Digite um nome acima para salvar nele, ou um novo para criar outra pasta. \n OBS: Evite acentuação e espaços!)\033[0m\n")
            else:
                print("\n\033[90mNenhum tópico criado ainda. O que você digitar será o primeiro!\033[0m\n")

            ## Coleta os inputs
            # Coleto o tópico
            nome_topico = input("\033[92m> \033[0mNome do tópico (ex: locomocao): ").strip()
            if not nome_topico:
                self._exibir_texto_com_bot("Aviso", "A criação foi cancelada (tópico vazio).")
                return

            # Coleto a aplicação
            nome_aplicacao = input("\033[92m> \033[0mNome da aplicação (ex: MeuRobo): ").strip()
            if not nome_aplicacao:
                self._exibir_texto_com_bot("Aviso", "A criação foi cancelada (nome vazio).")
                return

            tempo_simulacao = input("\033[92m> \033[0mTempo de simulação (em segundos): ").strip()
            nome_cena = input("\033[92m> \033[0mNome da cena (ex: cena_basica): ").strip()

            # 2. Prepara os caminhos e remove espaços/extensões
            nome_topico_limpo = nome_topico.replace(' ', '_').lower()
            nome_aplicacao_limpo = nome_aplicacao.replace('.py', '').replace(' ', '')
            nome_cena_limpo = nome_cena.replace('.ttt', '').replace(' ', '_')

            base_dir = Path.cwd()
            
            # Caminhos dos templates
            template_app = base_dir / "brainbyte" / "utils" / "basics" / "app.txt"
            template_scene = base_dir / "brainbyte" / "utils" / "basics" / "scene.ttt"
            
            # Caminhos de destino
            projects_dir = base_dir / "projects"
            sim_folder = projects_dir / nome_topico_limpo / nome_aplicacao_limpo
            
            # Garante que as pastas existam
            sim_folder.mkdir(parents=True, exist_ok=True)

            arquivo_py = f"{nome_aplicacao_limpo}.py"
            arquivo_ttt = f"{nome_cena_limpo}.ttt"

            caminho_novo_app = sim_folder / arquivo_py
            caminho_nova_cena = sim_folder / arquivo_ttt

            # 3. Gera o arquivo .py usando o template
            if not template_app.exists():
                raise FileNotFoundError(f"Template de app não encontrado: {template_app}")
                
            with open(template_app, 'r', encoding='utf-8') as f:
                conteudo_template = f.read()

            conteudo_final = conteudo_template.replace("{name_app}", nome_aplicacao_limpo)
            conteudo_final = conteudo_final.replace("{simulation_time}", tempo_simulacao)
            conteudo_final = conteudo_final.replace("{name_scene}", nome_cena_limpo)

            with open(caminho_novo_app, 'w', encoding='utf-8') as f:
                f.write(conteudo_final)

            # 4. Copia e renomeia o template da cena
            if not template_scene.exists():
                raise FileNotFoundError(f"Template de cena não encontrado: {template_scene}")
                
            shutil.copy2(template_scene, caminho_nova_cena)

            # 5. Feedback de sucesso e fala da bot
            mensagem_sucesso = (
                f"Simulação criada com sucesso!\n\n"
                f"📁 Projeto salvo em: projects/{nome_topico_limpo}/{nome_aplicacao_limpo}/\n"
                f"📄 Script principal: {arquivo_py}\n"
                f"📄 Cena do Coppelia: {arquivo_ttt}\n\n"
                f"O arquivo criado irá abrir para edições!"
            )
            self._exibir_texto_com_bot("Sucesso!", mensagem_sucesso)

            # 6. Abre o arquivo .py no editor padrão do sistema
            path_py_str = str(caminho_novo_app.resolve())
            try:
                if platform.system() == 'Windows':
                    os.startfile(path_py_str)
                elif platform.system() == 'Darwin': # macOS
                    subprocess.call(('open', path_py_str))
                else: # Linux
                    subprocess.call(('xdg-open', path_py_str))
            except Exception as e:
                self.logger.warning(f"Não foi possível abrir o arquivo automaticamente: {e}")

            # 7. Carrega a cena no CoppeliaSim silenciosamente (sem dar run)
            try:
                client = RemoteAPIClient()
                sim = client.require('sim')
                # O CoppeliaSim precisa do caminho absoluto para carregar a cena corretamente
                path_cena_str = str(caminho_nova_cena.resolve())
                sim.loadScene(path_cena_str)
                self.logger.info(f"Cena {arquivo_ttt} carregada no CoppeliaSim com sucesso.")
            except Exception as e:
                # Se o Coppelia não estiver aberto, ele avisa no log sem quebrar a criação
                self.logger.warning(f"CoppeliaSim não parece estar aberto para carregar a cena. Erro: {e}")

        except Exception as e:
            msg = traceback.format_exc()
            self.logger.error(f"Erro ao criar simulação: {msg}")
            self._exibir_texto_com_bot(
                "Erro Crítico", 
                f"Não foi possível criar a simulação:\n{e}\nTraceback:\n{msg}"
            )
        finally:
            # Esconde o cursor de volta pro menu continuar limpo
            sys.stdout.write('\033[?25l')
            sys.stdout.flush()

    # ---------- Funcionalidade para navegar no projeto ---------
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
                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                print(BOT_say("Navegador de projeto. Digite 'help' para ver comandos."))
                continue
            
            parts = cmd_input.split(maxsplit=1)
            command = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else ""
            
            if command in ("exit", "q", "quit"):
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
                print(BOT_say(f"Comando desconhecido: '{command}'. Digite 'help'.", width=50))
                current_depth = 1
            
            # Limpa a tela para próxima iteração (opcional, para manter a navegação limpa)
            # Se quiser manter histórico, remova os clears abaixo.
            os.system('cls' if os.name == 'nt' else 'clear')
            self.banner()
            print(BOT_say("Navegador de projeto. Digite 'help' para ver comandos."))

    def _show_nav_help(self):
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
        ignore_patterns = {'__init__.py','config.json','pyproject.toml','setup.py','requirements.txt','requirements-dev.txt', '__pycache__', '.git', '.venv', 'venv', 'env', '.idea', '.vscode', 'node_modules', 'build', 'dist'}
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
    def run(self): #A gui roda num loop while, enquanto o projeto está sendo executado.
        """Método principal: exibe menu principal e despacha ações."""
        intro_text = (
            "Bem-vindo ao BRAINBYTE! Eu sou o Blue, seu agente guia. "
            "Aqui você gerencia infraestrutura de robótica, organiza scripts e integra LLMs. "
            "Use as setas para navegar e Enter para selecionar."
        )
        while True:
            opcoes_principal = [
                "Iniciar simulação",      # 0
                "Criar nova simulação",   # 1 
                "Navegar pelo projeto",   # 2
                "Comandos",               # 3
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
            
            if escolha == -1 or escolha == 8:  # Sair
                # Limpa a tela para a despedida ficar limpa
                os.system('cls' if os.name == 'nt' else 'clear')
                self.banner()
                print(BOT_say("Até logo! Foi bom ajudar você.", width=40))
                break
            elif escolha == 0: # Inicia simulação
                self._choose_project()
            elif escolha == 1: # Criar nova simulação (Em desenvolvimento)
                '''self._exibir_texto_com_bot(
                    "Criar Nova Simulação",
                    "Módulo em desenvolvimento...\n"
                    "Em breve você poderá projetar novas simulações a partir daqui!"
                )'''
                self._create_new_simulation()
            elif escolha == 2: # Navegar pelo projeto
                self._navegate_project()
            elif escolha == 3: # Comandos
                self._exibir_texto_com_bot(
                    "Comandos Disponíveis",
                    "COMANDOS (a implementar):\n"
                    "  > run <script>  - executa um script\n"
                    "  > llm <prompt>   - consulta um modelo de linguagem\n"
                    "  > status         - mostra estado da infraestrutura"
                )
            elif escolha == 4:  # Ver Logs
                self._menu_logs()
            elif escolha == 5: # Configurações
                self._menu_configuracoes()
            elif escolha == 6: # Ajuda
                self._exibir_texto_com_bot(
                    "Ajuda",
                    "Aqui você encontra ajuda sobre as funcionalidades.\n"
                    "- Iniciar simulação: execute exemplos pré-programados.\n"
                    "- Criar nova simulação: crie suas próprias simulações.\n"
                    "- Comandos: lista de comandos disponíveis.\n"
                    "- Estrutura: mostra como o projeto está organizado.\n"
                    "- Configurações: ajuste opções do sistema.\n"
                    "- Ver Logs: exibe os últimos registros de execução."
                )
            elif escolha == 7: # Sobre o projeto
                self._exibir_texto_com_bot(
                    "Sobre o BRAINBYTE",
                    "BRAINBYTE - Gerenciador de Infraestrutura de Robótica\n\n"
                    "Funcionalidades:\n"
                    "• Organização de scripts de simulação\n"
                    "• Interface CLI amigável com mascote bot\n"
                    "• Configurações flexíveis (CLI/ROS)\n"
                    "• Estrutura modular pronta para expansão"
                )