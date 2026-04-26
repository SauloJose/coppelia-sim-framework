import os
import sys
from pathlib import Path
import shutil
import importlib
import textwrap
from brainbyte.utils.math import * 
from pynput import keyboard
import math

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

        # Filter low-pass configurations
        self.v_cmd = 0
        self.w_cmd = 0


        self.dt    = .05 #default 50 ms
        self.tau   = 0.3
        self.alpha = self.dt/self.tau  #time constant

    def _setup_output_filter(self, tau:float, dt):
        """
        Method to setup the configurations of the output low-pass filter
        
        v_out[k+1] = v_out[k] + (alpha)*(v_target - v_out[k])

        where alpha is approximaly dt/tau (1-e^(dt/tau))
        """
        self.tau = tau 
        self.dt  = dt

        if self.tau > 0:
            self.alpha = 1.0 - math.exp(-self.dt / self.tau)
        else:
            self.alpha = 1.0 # Sem filtro (resposta instantânea)

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

        #Aplicação do filtro de saída
        return self.output_filter(v,w)

    def output_filter(self, v, w):
        """ Filter the output"""
        self.v_cmd += self.alpha*(v - self.v_cmd)
        self.w_cmd += self.alpha*(w - self.w_cmd)
        return self.v_cmd, self.w_cmd 
    

    def stop(self):
        """Para o listener (deve ser chamado ao final da simulação)."""
        if self._listener.running:
            self._listener.stop()

