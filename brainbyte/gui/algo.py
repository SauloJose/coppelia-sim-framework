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

    # (Mantenha _exibir_texto_com_raposa, _list_examples, _choose_example e _generate_tree inalterados)

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
                "Sair"
            ]
            escolha = self._menu_navegavel(
                "MENU PRINCIPAL",
                opcoes_principal,
                msg_raposa=intro_text,
                subtitulo=None
            )
            
            if escolha == -1 or escolha == 6:  # Sair
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
                    "- Configurações: ajuste opções do sistema."
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