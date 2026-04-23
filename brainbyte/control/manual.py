import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap
from brainbyte.utils.math import * 
from pynput import keyboard


class KeyboardController:
    """
    Captura teclas (WASD e setas) de forma assíncrona e não-bloqueante.
    Funciona em Windows, Linux e macOS sem necessidade de root.
    """
    def __init__(self, v_max=0.5, w_max=1.0):
        self.v_max = v_max
        self.w_max = w_max
        
        # Conjunto de teclas atualmente pressionadas
        self._pressed = set()
        
        # Inicia o listener em uma thread separada
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self._listener.start()

    def _on_press(self, key):
        """Callback chamado quando uma tecla é pressionada."""
        try:
            # Tecla com caractere (ex: 'w', 'a', 's', 'd')
            self._pressed.add(key.char.lower() if key.char else None)
        except AttributeError:
            # Tecla especial (setas, shift, etc.)
            self._pressed.add(key)

    def _on_release(self, key):
        """Callback chamado quando uma tecla é solta."""
        try:
            self._pressed.discard(key.char.lower() if key.char else None)
        except AttributeError:
            self._pressed.discard(key)

    def get_command(self):
        """
        Retorna (velocidade_linear, velocidade_angular) baseado nas teclas pressionadas.
        Combinações são suportadas (ex: W + D → frente + giro à direita).
        """
        v = 0.0
        w = 0.0

        # Movimento linear (frente/trás)
        if 'w' in self._pressed or keyboard.Key.up in self._pressed:
            v += self.v_max
        if 's' in self._pressed or keyboard.Key.down in self._pressed:
            v -= self.v_max

        # Movimento angular (rotação)
        if 'a' in self._pressed or keyboard.Key.left in self._pressed:
            w += self.w_max
        if 'd' in self._pressed or keyboard.Key.right in self._pressed:
            w -= self.w_max

        return v, w

    def stop(self):
        """Para o listener (deve ser chamado ao final da simulação)."""
        if self._listener.running:
            self._listener.stop()

