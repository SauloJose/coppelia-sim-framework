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
        cum_error += error * dt
        return cum_error, self.ki * cum_error

    def _calc_derivative(self, error, dt):
        """Termo derivativo (derivada do erro)."""
        if dt > 0:
            derivative = (error - self.last_error) / dt
            self.last_error = error
            return self.kd * derivative
        else:
            return np.zeros_like(error)
        

    def run(self, y, u_min=-1, u_max=1):
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
        u_min = self._make_compatible(u_min)
        u_max = self._make_compatible(u_max)

        # Erro atual
        self.error = self.set_point - y

        # Termo proporcional
        P = self._calc_proportional(self.error)

        #Termo derivativo
        D = self._calc_derivative(self.error, dt)

        # Termo integrativo
        new_cum, I = self._calc_integral(self.error, dt)

        u_unsat = P+I+D

        # Lógica de Clamping compatível com Escalares e Arrays
        # Se (Saturado em cima E erro positivo) OU (Saturado em baixo E erro negativo)
        is_saturated_high = (u_unsat > u_max) & (self.error > 0)
        is_saturated_low  = (u_unsat < u_min) & (self.error < 0)
        dont_integrate = is_saturated_high | is_saturated_low

        if isinstance(self.error, np.ndarray):
            # Se for array, usamos np.where para decidir junta por junta
            self.cum_error = np.where(dont_integrate, self.cum_error, new_cum)
        else:
            # Se for escalar, usamos o if comum
            if not dont_integrate:
                self.cum_error = new_cum

        self.output = np.clip(u_unsat, u_min, u_max)
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
    
class DifferentialController:
    def __init__(self, pos_init: np.ndarray,
                 set_point: np.ndarray,
                 k_rho: float,
                 k_alpha: float,
                 k_beta: float,
                 dt: float = 0.05):
        
        # Ganhos do controlador
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

        # Estados
        self.set_point = set_point    
        self.current_state = pos_init 

        # Saídas de comando (inicializadas sem underscore para consistência)
        self.v_cmd = 0.0
        self.w_cmd = 0.0 

        self.v_max = 1
        self.a_max = 3
        self.w_max = 6
        self.a_max = 6
        
    def set_SP(self, set_point):
        """
        Define um novo objetivo (Set Point) para o controlador perseguir.
        """
        self.set_point = set_point

    @staticmethod
    @njit
    def _calc_logic(actual, set_point):
        """
        Cálculo puramente matemático das coordenadas polares de erro.
        Usa @njit para ser compilado em código de máquina e rodar em microsegundos.
        
        Calcula:
        - rho: Distância Euclidiana $\sqrt{\Delta x^2 + \Delta y^2}$
        - alpha: Ângulo entre a frente do robô e a linha do objetivo
        - beta: Ângulo entre a linha do objetivo e a orientação final
        """
        dx = set_point[0] - actual[0]
        dy = set_point[1] - actual[1]
        theta = actual[2]

        rho = np.sqrt(dx**2 + dy**2)
        
        # Alpha: ângulo para o objetivo em relação à frente do robô
        alpha = normalize_angle(np.arctan2(dy, dx) - theta)
        
        # Beta: ajuste da orientação final
        beta = normalize_angle(set_point[2] - theta - alpha)
        
        return rho, alpha, beta

    def set_parameters(self, k_rho, k_alpha, k_beta):
        """
        Permite atualizar os ganhos dinamicamente e valida as condições de estabilidade
        de Lyapunov para evitar que o robô se comporte de forma errática.
        """
        # Condição de estabilidade: k_rho > 0, k_beta < 0, k_alpha > k_rho
        if k_rho <= 0 or k_beta >= 0:
            print("Aviso: Ganhos podem não garantir estabilidade (Recomendado: k_rho > 0, k_beta < 0)")
        
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
    
    def set_max_values(self, 
                       v_max =1.0, 
                       a_max = 4.0, 
                       w_max = 10.0, 
                       alpha_max =4.0):
        self.v_max = v_max
        self.a_max = a_max 
        self.w_max = w_max 
        self.alpha_max = alpha_max 

    def get_control(self, actual_point: np.ndarray, dt: float = 0.05):
        actual_point = np.asarray(actual_point)
        rho, alpha, beta = self._calc_logic(actual_point, self.set_point)
        
        rho_tol = 0.05
        theta_tol = 0.05
        
        direction = 1.0
        if alpha > np.pi/2 or alpha < -np.pi/2:
            direction = -1.0
            alpha = normalize_angle(alpha + np.pi)
            beta = normalize_angle(beta + np.pi)

        # 1. Velocidades Brutas baseadas no erro
        v_target = direction * self.k_rho * rho
        w_target = self.k_alpha * alpha + self.k_beta * beta
        
        # 2. Tratamento de Chegada (sua lógica original mantida)
        if rho < rho_tol:
            v_target = 0.0
            error_theta = normalize_angle(self.set_point[2] - actual_point[2])
            if abs(error_theta) < theta_tol:
                w_target = 0.0
                self.v_cmd, self.w_cmd = 0.0, 0.0
                return 0.0, 0.0
            else:
                w_target = 0.5 * error_theta

        # 3. SATURAÇÃO DE VELOCIDADE (Limite máximo do motor)
        v_target = np.clip(v_target, -self.v_max, self.v_max)
        w_target = np.clip(w_target, -self.w_max, self.w_max)

        # 4. SLEW RATE (Limite de Aceleração) - A "Mágica" que substitui o filtro
        # Em vez de um filtro, dizemos: "A velocidade só pode mudar X por ciclo"
        max_dv = self.a_max * dt
        max_dw = self.alpha_max * dt # alpha_max aqui é aceleração angular

        # Aplica o limite na variação da velocidade
        dv = np.clip(v_target - self.v_cmd, -max_dv, max_dv)
        dw = np.clip(w_target - self.w_cmd, -max_dw, max_dw)

        # Atualiza o estado de comando interno
        self.v_cmd += dv
        self.w_cmd += dw

        return self.v_cmd, self.w_cmd