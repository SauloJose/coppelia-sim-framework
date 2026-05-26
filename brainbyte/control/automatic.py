# ===========================================================================
# Scripts para controladores úteis no sistema
#
#
# ===========================================================================
from brainbyte.utils.math import * 
 
import numpy as np

class PID_Controller:
    """
    Controlador PID Profissional com suporte a escalares e arrays (broadcasting)
    e sistema de anti-windup (Clamping condicional).
    """
    def __init__(self, var, kp, ki, kd, dt, set_point):
        """
        Inicializa o controlador PID.

        Parâmetros:
        -----------
        var : float ou np.ndarray
            Exemplo da variável a ser controlada (define as dimensões do controlador).
        kp, ki, kd : float ou np.ndarray
            Ganhos proporcional, integral e derivativo. 
        dt : float
            Intervalo de tempo nominal da simulação.
        set_point : float ou np.ndarray
            Valor de referência (setpoint) desejado.
        """
        # Determina as dimensões a partir do exemplo 'var'
        self.dim_control = np.asarray(var).shape if isinstance(var, np.ndarray) else ()

        # Converte ganhos para formato compatível (arrays ou escalares)
        self.kp = self._make_compatible(kp)
        self.ki = self._make_compatible(ki)
        self.kd = self._make_compatible(kd)
        self.set_point = self._make_compatible(set_point)
        self.dt = dt
        
        # Inicia o estado interno
        self.reset()

    def _make_compatible(self, value):
        """Garante que o valor tenha o formato esperado para broadcasting do Numpy."""
        if self.dim_control:
            if np.isscalar(value):
                return np.full(self.dim_control, value, dtype=float)
            else:
                val_array = np.asarray(value, dtype=float)
                # Verifica compatibilidade de broadcasting silenciosamente
                return np.broadcast_to(val_array, self.dim_control)
        return float(value)
        
    def set_setpoint(self, new_sp):
        """Define um novo valor de setpoint (referência)."""
        self.set_point = self._make_compatible(new_sp)

    def reset(self):
        """Reinicia os estados internos do controlador (erros acumulados e passado)."""
        shape = self.dim_control
        self.error = np.zeros(shape) if shape else 0.0
        self.last_error = np.zeros(shape) if shape else 0.0
        self.cum_error = np.zeros(shape) if shape else 0.0
        self.output = np.zeros(shape) if shape else 0.0

    def run(self, y, u_min=-1.0, u_max=1.0):
        """
        Executa uma iteração do controlador PID.

        Parâmetros:
        -----------
        y : float ou np.ndarray
            Valor medido atual.
        u_min, u_max : float ou np.ndarray
            Limites mínimo e máximo de saturação do sinal de controle.

        Retorna:
        --------
        u : float ou np.ndarray
            Sinal de controle saturado.
        """
        y = self._make_compatible(y)
        u_min = self._make_compatible(u_min)
        u_max = self._make_compatible(u_max)

        # 1. Cálculo do erro atual
        self.error = self.set_point - y

        # 2. Termo Proporcional
        P = self.kp * self.error

        # 3. Termo Derivativo
        if self.dt > 0:
            derivative = (self.error - self.last_error) / self.dt
        else:
            derivative = np.zeros(self.dim_control) if self.dim_control else 0.0
        D = self.kd * derivative

        # 4. Termo Integral (Integração provisória para avaliar Clamping)
        delta_integral = self.error * self.dt
        provisional_cum_error = self.cum_error + delta_integral
        I = self.ki * provisional_cum_error

        # 5. Cálculo do controle não saturado
        u_unsat = P + I + D

        # 6. Lógica de Clamping (Anti-Windup)
        # Verifica se estourou os limites E o erro atual está empurrando mais ainda na direção da saturação
        is_saturated_high = (u_unsat > u_max) & (self.error > 0)
        is_saturated_low  = (u_unsat < u_min) & (self.error < 0)
        dont_integrate = is_saturated_high | is_saturated_low

        # 7. Atualização dos Estados (Apenas se não cair no clamping)
        if isinstance(self.error, np.ndarray):
            self.cum_error = np.where(dont_integrate, self.cum_error, provisional_cum_error)
        else:
            if not dont_integrate:
                self.cum_error = provisional_cum_error

        self.last_error = self.error

        # 8. Saída final saturada
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

        # invert front
        self.inverted = False
        
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
        self.inverted = False
        if alpha > np.pi/2 or alpha < -np.pi/2:
            direction = -1.0
            self.inverted = True
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
    

class OmnidirectionalController:
    def __init__(self, pos_init: np.ndarray,
                 set_point: np.ndarray,
                 k_x: float,
                 k_y: float,
                 k_theta: float,
                 dt: float = 0.05):
        
        # Ganhos do controlador
        self.k_x = k_x
        self.k_y = k_y
        self.k_theta = k_theta

        # Estados
        self.set_point = np.asarray(set_point, dtype=np.float64)
        self.current_state = np.asarray(pos_init, dtype=np.float64) 
        self.dt = dt

        # Comandos internos
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.w_cmd = 0.0 

        # Limites
        self.v_max = 1.0
        self.w_max = 6.0
        self.a_max = 3.0
        self.alpha_max = 6.0
        
    def set_SP(self, set_point):
        self.set_point = np.asarray(set_point, dtype=np.float64)

    @staticmethod
    @njit
    def _calc_errors(actual, set_point):
        """
        Cálculo puramente Cartesiano (Holonômico).
        Retorna os erros nos eixos globais e o erro de orientação normalizado.
        """
        dx = set_point[0] - actual[0]
        dy = set_point[1] - actual[1]
        
        # Correção: Normalização do ângulo para o robô não dar giros bobos de 360°
        dtheta = set_point[2] - actual[2]
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        
        return dx, dy, dtheta

    def set_parameters(self, k_x, k_y, k_theta):
        self.k_x = k_x
        self.k_y = k_y
        self.k_theta = k_theta
    
    def set_max_values(self, v_max=1.0, a_max=4.0, w_max=10.0, alpha_max=4.0):
        self.v_max = v_max
        self.a_max = a_max 
        self.w_max = w_max 
        self.alpha_max = alpha_max 

    def get_control(self, actual_point: np.ndarray, dt: float = None):
        if dt is None:
            dt = self.dt
            
        actual_point = np.asarray(actual_point, dtype=np.float64)
        dx, dy, dtheta = self._calc_errors(actual_point, self.set_point)
        
        # Distância Euclidiana para critério de parada
        rho = np.sqrt(dx**2 + dy**2)
        
        rho_tol = 0.05
        theta_tol = 0.05

        # 1. Velocidades Globais baseadas no erro (Controle Proporcional)
        vx_global_target = self.k_x * dx
        vy_global_target = self.k_y * dy
        w_target = self.k_theta * dtheta

        # 2. Rotação do Mundo para o Robô (Transformação de Referencial)
        theta = actual_point[2]
        c = np.cos(theta)
        s = np.sin(theta)

        vx_local_target = vx_global_target * c + vy_global_target * s
        vy_local_target = -vx_global_target * s + vy_global_target * c
        
        # 3. Tratamento de Chegada Suave
        # Em vez de zerar no tranco, zeramos o ALVO e deixamos o Slew Rate frear o robô respeitando a_max
        if rho < rho_tol:
            vx_local_target = 0.0
            vy_local_target = 0.0
        
        if abs(dtheta) < theta_tol:
            w_target = 0.0

        # Condição de parada real: erro dentro da tolerância E robô efetivamente parado
        if rho < rho_tol and abs(dtheta) < theta_tol and abs(self.vx_cmd) < 0.01 and abs(self.vy_cmd) < 0.01 and abs(self.w_cmd) < 0.01:
            self.vx_cmd, self.vy_cmd, self.w_cmd = 0.0, 0.0, 0.0
            return np.array([0.0, 0.0]), 0.0

        # 4. Saturação de Velocidade Linear (Vetor 2D)
        v_vector_target = np.array([vx_local_target, vy_local_target])
        v_norm = np.linalg.norm(v_vector_target)
        if v_norm > self.v_max:
            v_vector_target = v_vector_target * (self.v_max / v_norm)
        
        w_target = np.clip(w_target, -self.w_max, self.w_max)

        # 5. SLEW RATE VETORIAL (Correção da aceleração)
        max_dv = self.a_max * dt
        max_dw = self.alpha_max * dt 

        # Calculamos a diferença vetorial de velocidade linear
        dv_linear = v_vector_target - np.array([self.vx_cmd, self.vy_cmd])
        dv_norm = np.linalg.norm(dv_linear)
        
        # Limita a aceleração mantendo a direção correta do movimento
        if dv_norm > max_dv and dv_norm > 0:
            dv_linear = dv_linear * (max_dv / dv_norm)

        dw = np.clip(w_target - self.w_cmd, -max_dw, max_dw)

        # Atualiza o estado interno de comandos
        self.vx_cmd += dv_linear[0]
        dvy_linear = dv_linear[1]
        self.vy_cmd += dvy_linear 
        self.w_cmd += dw

        return np.array([self.vx_cmd, self.vy_cmd]), self.w_cmd
    
class SimpleController:
    def __init__(self, k_rho=0.8, k_alpha=1.5, v_max=0.5, w_max=1.0, tolerance=0.05):
        # Ganhos Proporcionais
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        
        # Limites dinâmicos
        self.v_max = v_max
        self.w_max = w_max
        
        # Tolerância de chegada para evitar a "Singularidade da Origem" (girar como pião)
        self.tolerance = tolerance
        
        # Atributo essencial para evitar erros de escopo ao clicar no mapa
        self.set_point = np.array([0.0, 0.0, 0.0])

    def set_max_values(self, v_max=1.0, w_max=4.0, *args, **kwargs):
        """Mantém a compatibilidade com a chamada padrão do TurtleBot."""
        self.v_max = v_max
        self.w_max = w_max

    def get_control(self, actual_pos, target_point):
        """Calcula comandos simples focados estritamente em avançar de frente."""
        self.set_point = np.asarray(target_point)
        
        dx = self.set_point[0] - actual_pos[0]
        dy = self.set_point[1] - actual_pos[1]
        theta = actual_pos[2]

        rho = np.hypot(dx, dy)
        
        # ZONA MORTA: Se estiver muito perto do alvo, desliga os motores e evita o giro infinito
        if rho < self.tolerance:
            return 0.0, 0.0

        # Ângulo direto para o alvo em relação à frente do robô
        alpha = np.arctan2(dy, dx) - theta
        
        # Normalização do ângulo estritamente entre [-pi, pi]
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        # Suavização da transição: 
        # Em vez de um corte brusco (if > 60 graus), aplicamos uma redução 
        # gradual da velocidade linear conforme o robô perde o alinhamento.
        
        if abs(alpha) > (np.pi / 2):
            # Se o alvo estiver a mais de 90 graus (atrás), só gira
            v_target = 0.0
            w_target = self.k_alpha * alpha
        else:
            # Controle proporcional. O cosseno(alpha) faz a velocidade linear 
            # diminuir naturalmente em curvas fechadas e aumentar em retas.
            v_target = self.k_rho * rho * np.cos(alpha)
            w_target = self.k_alpha * alpha

        # Saturação de segurança
        v_cmd = np.clip(v_target, 0.0, self.v_max) 
        w_cmd = np.clip(w_target, -self.w_max, self.w_max)

        return v_cmd, w_cmd