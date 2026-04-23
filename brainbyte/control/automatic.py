# ===========================================================================
# Scripts para controladores úteis no sistema
#
#
# ===========================================================================
from brainbyte.utils.math import * 

# Controlador PID 
class PID_Controller:
    def __init__(self, var, kp, ki, kd, dt, set_point):
        """
        Inicializa o controlador PID.

        Parâmetros:
        -----------
        var : float ou np.ndarray
            Exemplo da variável a ser controlada, usado para definir as dimensões do controlador.
        kp, ki, kd : float ou np.ndarray
            Ganhos proporcional, integral e derivativo. Podem ser escalares ou ter o mesmo shape de `var`.
        set_point : float ou np.ndarray
            Valor de referência (setpoint) desejado para a variável controlada.
        """
        # Determina as dimensões a partir do exemplo 'var'
        self.dim_control = np.asarray(var).shape if isinstance(var, np.ndarray) else ()

        # Converte ganhos para formato compatível
        self.kp = self._make_compatible(kp)
        self.ki = self._make_compatible(ki)
        self.kd = self._make_compatible(kd)
        self.set_point = self._make_compatible(set_point)

        self.dt = dt
        # Estado interno
        self.reset()

    def _make_compatible(self, value):
        """Garante que o valor tenha o formato esperado para broadcasting."""
        if self.dim_control:
            # Variável controlada é array
            if np.isscalar(value):
                return np.full(self.dim_control, value)
            else:
                value = np.asarray(value)
                # Verifica se o shape é compatível (permite broadcasting)
                np.broadcast_to(value, self.dim_control)
                return value
        else:
            # Variável controlada é escalar
            return value
        
    def set_setpoint(self, new_sp):
        """
        Define um novo valor de setpoint (referência).
        """
        self.set_point = self._make_compatible(new_sp)

    def reset(self):
        """
        Reinicia o estado interno do controlador (erros acumulados, derivativo, tempo).
        """
        shape = self.dim_control
        self.error = np.zeros(shape) if shape else 0.0
        self.last_error = np.zeros(shape) if shape else 0.0
        self.cum_error = np.zeros(shape) if shape else 0.0
        self.output = np.zeros(shape) if shape else 0.0

    def _calc_proportional(self, error):
        """Termo proporcional."""
        return self.kp * error

    def _calc_integral(self, error, dt):
        """Termo integral com anti-windup simples (limitação não incluída)."""
        self.cum_error += error * dt
        return self.ki * self.cum_error

    def _calc_derivative(self, error, dt):
        """Termo derivativo (derivada do erro)."""
        if dt > 0:
            derivative = (error - self.last_error) / dt
            self.last_error = error
            return self.kd * derivative
        else:
            return np.zeros_like(error)
        

    def run(self, y):
        """
        Executa uma iteração do controlador.

        Parâmetros:
        -----------
        y : float ou np.ndarray
            Valor medido atual (mesmo formato da variável controlada).
        dt : float
            Intervalo de tempo desde a última iteração (passo da simulação).

        Retorna:
        --------
        u : float ou np.ndarray
            Sinal de controle.
        """
        dt = self.dt
        y = self._make_compatible(y)

        # Erro atual
        self.error = self.set_point - y

        # Termo proporcional
        P = self._calc_proportional(self.error)

        #Termo integral
        I = self._calc_integral(self.error, dt)

        #Termo derivativo
        D = self._calc_derivative(self.error, dt)

        #Atualiza o último erro
        self.last_error = self.error.copy() if isinstance(self.error, np.ndarray) else self.error

        # Saída total
        self.output = P + I +D

        return self.output 


class On_Off_Controller:
    def __init__(self, 
                 var, 
                 set_point,
                 u_max, 
                 u_min=0.0, 
                 hysteresis=0.0):
        """
        Inicializa o controlador On-Off (Bang-Bang).

        Parâmetros:
        -----------
        var : float ou np.ndarray
            Exemplo da variável a ser controlada, usado para definir as dimensões do controlador.
        set_point : float ou np.ndarray
            Valor de referência (setpoint) desejado.
        u_max : float ou np.ndarray
            Sinal de controle máximo (Ligado / Para frente).
        u_min : float ou np.ndarray
            Sinal de controle mínimo (Desligado / Para trás). Padrão é 0.0.
        hysteresis : float ou np.ndarray
            Banda de tolerância ao redor do setpoint para evitar chattering (vibração).
        """
        self.dim_control = np.asarray(var).shape if isinstance(var, np.ndarray) else ()

        # Converte parâmetros para formato compatível
        self.set_point = self._make_compatible(set_point)
        self.u_max = self._make_compatible(u_max)
        self.u_min = self._make_compatible(u_min)
        self.hysteresis = self._make_compatible(hysteresis)

    def _make_compatible(self, value):
        """Garante que o valor tenha o formato esperado para broadcasting."""
        if self.dim_control:
            if np.isscalar(value):
                return np.full(self.dim_control, value)
            else:
                value = np.asarray(value)
                np.broadcast_to(value, self.dim_control)
                return value
        else:
            return value
    
    def set_setpoint(self,new_sp):
        """ Defino um novo valor de setpoint (referência)"""
        self.set_point = self._make_compatible(new_sp)

    def reset(self):
        """ Reinicia o estado da saída do controlador"""
        shape = self.dim_control
        self.output = np.zeros(shape) if shape else 0.0

    def run(self, y):
        """
        Executa uma iteração do controlador On-Off.

        Parâmetros:
        -----------
        y : float ou np.ndarray
            Valor medido atual.
        dt : float, opcional
            Intervalo de tempo. Mantido na assinatura para compatibilidade 
            com outros controladores (como o PID), mas não é usado no On-Off.

        Retorna:
        --------
        u : float ou np.ndarray
            Sinal de controle (u_max ou u_min).
        """
        y = self._make_compatible(y)
        error = self.set_point - y

        # Limites da histerese
        limite_sup = self.hysteresis / 2.0
        limite_inf = -self.hysteresis / 2.0

        if self.dim_control:
            # Lógica para arrays (numpy vectorization)
            self.output = np.where(error > limite_sup, self.u_max, 
                          np.where(error < limite_inf, self.u_min, self.output))
        else:
            # Lógica para escalares
            if error > limite_sup:
                self.output = self.u_max
            elif error < limite_inf:
                self.output = self.u_min
            # Se o erro estiver dentro da banda de histerese, a saída anterior é mantida

        return self.output