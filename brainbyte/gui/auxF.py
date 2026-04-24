import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap

# Cores ANSI
ORANGE = "\033[38;5;208m"
BLACK = "\033[30m"
BOLD = "\033[1m"
BLUE = '\033[94m'  # Azul claro clássico
CYAN = '\033[96m'  # Ciano (Um azul mais vibrante e tecnológico, tipo "neon")
RESET = "\033[0m"


def get_key():
    """Captura uma tecla pressionada de forma cross-platform."""
    if os.name == 'nt':
        import msvcrt
        key = msvcrt.getch()
        # Teclas especiais (setas) retornam dois bytes: \xe0 + código
        if key == b'\xe0':
            key = msvcrt.getch()
            return {
                b'H': 'UP',
                b'P': 'DOWN',
                b'K': 'LEFT',
                b'M': 'RIGHT'
            }.get(key, None)
        elif key == b'\r':
            return 'ENTER'
        elif key == b' ':
            return 'SPACE'
        else:
            return key.decode('utf-8', errors='ignore').lower()
    else:
        import termios, tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # ESC sequence
                ch += sys.stdin.read(2)
                return {
                    '\x1b[A': 'UP',
                    '\x1b[B': 'DOWN',
                    '\x1b[C': 'RIGHT',
                    '\x1b[D': 'LEFT'
                }.get(ch, None)
            elif ch == '\r':
                return 'ENTER'
            elif ch == ' ':
                return 'SPACE'
            else:
                return ch.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ---------- Arte do Robô ----------
BOT_ART =  r"""
      ██    ██
   ██████████████
 ████░░░░░░░░░░████
   ██░░  ░░  ░░██
 ████░░░░░░░░░░████
   ██████████████
      ██    ██
"""


def BOT_say(message, width=50):
    """Retorna o robô com um balão de fala ao lado dele."""
    
    # 1. Prepara as linhas de texto
    lines = []
    for paragraph in message.splitlines():
        if not paragraph.strip():
            lines.append("") # Mantém linhas em branco
        else:
            lines.extend(textwrap.wrap(paragraph, width))
            
    max_len = max((len(line) for line in lines), default=0)
    
    # 2. Constrói o balão de fala (agora lateral)
    box_lines = []
    box_lines.append("   ╭" + "─" * (max_len + 2) + "╮")
    
    # Define a linha do meio para colocar a setinha apontando para o robô
    pointer_idx = len(lines) // 2 
    
    for i, line in enumerate(lines):
        content = line.ljust(max_len)
        if i == pointer_idx:
            box_lines.append(f" ◀─┤ {content} │")
        else:
            box_lines.append(f"   │ {content} │")
            
    box_lines.append("   ╰" + "─" * (max_len + 2) + "╯")
    
    # 3. Prepara as linhas do robô
    # Removemos apenas as quebras de linha vazias no início e fim
    robot_lines = BOT_ART.strip('\n').split('\n')
    robot_width = max((len(l) for l in robot_lines), default=0)
    
    # 4. Alinha Robô e Balão verticalmente (centralizados)
    diff = len(robot_lines) - len(box_lines)
    if diff > 0:
        box_offset = diff // 2
        robot_offset = 0
    else:
        box_offset = 0
        robot_offset = (-diff) // 2
        
    total_lines = max(len(robot_lines), len(box_lines))
    
    # 5. Junta as duas colunas
    final_result = []
    for i in range(total_lines):
        # Renderiza linha do robô
        r_idx = i - robot_offset
        if 0 <= r_idx < len(robot_lines):
            # ljust preenche com espaços para manter o retângulo perfeito
            r_line = f"{CYAN}{robot_lines[r_idx].ljust(robot_width)}{RESET}"
        else:
            r_line = " " * robot_width
            
        # Renderiza linha do balão
        b_idx = i - box_offset
        if 0 <= b_idx < len(box_lines):
            b_line = f"{BOLD}{box_lines[b_idx]}{RESET}"
        else:
            b_line = ""
            
        # Concatena a esquerda (Robô) com a direita (Balão)
        final_result.append(f"{r_line}{b_line}")
        
    return "\n".join(final_result)


def BOT_print(message, width=50):
    """Imprime a raposa falando a mensagem."""
    # Garante que as cores funcionem no Windows (se for o caso)
    if os.name == 'nt':
        os.system('color')
    print("\n" + BOT_say(message, width) + "\n")