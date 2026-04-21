import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap

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
  ██████
██████████
██▓▓██▓▓██
██████████
  ██  ██
"""

# Balão de fala formatado
def fox_say(message, width=60):
    """Retorna a raposa com um balão de fala contendo a mensagem."""
    lines = textwrap.wrap(message, width)
    max_len = max(len(line) for line in lines) if lines else 0
    balloon_top = "   " + "_" * (max_len + 2)
    balloon_bottom = "   " + "-" * (max_len + 2)
    
    result = [balloon_top]
    for i, line in enumerate(lines):
        if len(lines) == 1:
            result.append(f"  < {line.ljust(max_len)} >")
        elif i == 0:
            result.append(f"  / {line.ljust(max_len)} \\")
        elif i == len(lines) - 1:
            result.append(f"  \\ {line.ljust(max_len)} /")
        else:
            result.append(f"  | {line.ljust(max_len)} |")
    result.append(balloon_bottom)
    result.append(FOX_ART)
    return "\n".join(result)

def fox_print(message, width=60):
    """Imprime a raposa falando a mensagem."""
    print(fox_say(message, width))