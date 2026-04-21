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

# ---------- Arte da Raposa ----------
FOX_ART = r"""
    █    █  
   ██    ██
  ██████████
████  ██  ████
   ████████
     ████
"""

def fox_say(message, width=50):
    """Retorna a raposa com um balão de fala elegante e colorido."""
    lines = textwrap.wrap(message, width)
    max_len = max(len(line) for line in lines) if lines else 0
    
    # Construção do balão com bordas arredondadas (Unicode)
    top =    "  ╭" + "─" * (max_len + 2) + "╮"
    bottom = "  ╰" + "─" * (max_len + 2) + "╯"
    pointer = "   " + " " * (max_len // 4) + "▼"
    
    result = [f"{BOLD}{top}{RESET}"]
    
    for line in lines:
        content = line.ljust(max_len)
        result.append(f"{BOLD}  │ {RESET}{content}{BOLD} │{RESET}")
    
    result.append(f"{BOLD}{bottom}{RESET}")
    result.append(pointer)
    result.append(f"{ORANGE}{FOX_ART}{RESET}")
    
    return "\n".join(result)

def fox_print(message, width=50):
    """Imprime a raposa falando a mensagem."""
    # Garante que as cores funcionem no Windows (se for o caso)
    if os.name == 'nt':
        os.system('color')
    print(fox_say(message, width))
