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

@njit
def normalize_angle(angle):
    """Normaliza ângulo para [-pi, pi]"""
    return np.arctan2(np.sin(angle), np.cos(angle))

class DifferentialController:
    def __init__(self, pos_init: np.ndarray,
                 set_point: np.ndarray,
                 k_rho: float,
                 k_alpha: float,
                 k_beta: float,
                 dt: float = 0.05):
        
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

        self.set_point = set_point    
        self.current_state = pos_init 

        self.v_cmd = 0.0
        self.w_cmd = 0.0 
        self.inverted = False
        
    def set_SP(self, set_point):
        """Define um novo objetivo (Set Point) para o controlador perseguir."""
        self.set_point = set_point

    @staticmethod
    @njit
    def _calc_logic(actual, set_point):
        """Cálculo das coordenadas polares de erro."""
        dx = set_point[0] - actual[0]
        dy = set_point[1] - actual[1]
        theta = actual[2]
        theta_des = set_point[2]

        # Distância ao objetivo
        rho = np.sqrt(dx**2 + dy**2)
        
        # Ângulo para o objetivo em relação ao frame global
        goal_angle = np.arctan2(dy, dx)
        
        # Alpha: ângulo para o objetivo em relação à frente do robô
        alpha = normalize_angle(goal_angle - theta)
        
        # Beta: orientação desejada relativa ao vetor goal_angle
        beta = normalize_angle(theta_des - goal_angle)
        
        return rho, alpha, beta
    
    def set_max_values(self, v_max, w_max):
        pass 
    
    def set_parameters(self, k_rho, k_alpha, k_beta):
        """Atualiza os ganhos e valida condições de estabilidade."""
        if k_rho <= 0:
            print("Aviso: k_rho deve ser > 0 para estabilidade")
        if k_beta < 0:
            print("Aviso: k_beta > 0 geralmente recomendado para estabilidade")
        if k_alpha - k_rho <= 0:
            print("Aviso: k_alpha > k_rho recomendado para estabilidade")
        
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

    def get_control(self, actual_point: np.ndarray, dt: float = 0.05):
        """
        Calcula comandos de velocidade para o robô diferencial.
        """
        actual_point = np.asarray(actual_point, dtype=np.float64)
        rho, alpha, beta = self._calc_logic(actual_point, self.set_point)
        
        # Tolerâncias para critério de parada
        rho_tol = 0.05      # 5cm
        theta_tol = 0.05    # ~3 graus
        
        # Verifica critério de parada primeiro
        if rho < rho_tol:
            # Erro de orientação final
            error_theta = normalize_angle(self.set_point[2] - actual_point[2])
            
            if abs(error_theta) < theta_tol:
                # Objetivo completamente alcançado
                self.v_cmd = 0.0
                self.w_cmd = 0.0
                return 0.0, 0.0
            else:
                # Apenas corrige orientação (controle puro de rotação)
                self.v_cmd = 0.0
                self.w_cmd = self.k_alpha * error_theta
                return self.v_cmd, self.w_cmd
        else:
            # Implementação simplificada e estável:
            v_target = self.k_rho * rho * np.cos(alpha)
            
            # Termo de rotação com saturação suave para evitar singularidade em alpha=0
            if abs(alpha) > 0.001:
                sinc_term = np.sin(alpha) / alpha
            else:
                sinc_term = 1.0  # limite quando alpha -> 0
                
            w_target = self.k_alpha * alpha + self.k_rho * sinc_term * np.cos(alpha) * (alpha + self.k_beta * beta)

        # Atualiza diretamente as velocidades sem filtros de aceleração ou clip de velocidade máxima
        self.v_cmd = v_target
        self.w_cmd = w_target

        return self.v_cmd, self.w_cmd
    
    def get_state(self):
        """Retorna o estado atual do controlador para debug"""
        return {
            'set_point': self.set_point,
            'v_cmd': self.v_cmd,
            'w_cmd': self.w_cmd,
            'inverted': self.inverted,
            'gains': (self.k_rho, self.k_alpha, self.k_beta)
        }


class OmnidirectionalController:
    def __init__(self, set_point: np.ndarray,
                 k_x: float,
                 k_y: float,
                 k_theta: float,
                 rho_tol: float = 0.05,    # Tolerância linear padrão (ex: 5 cm)
                 theta_tol: float = 0.05): # Tolerância angular padrão (ex: ~2.8 graus)
        
        # Ganhos do controlador
        self.k_x = k_x
        self.k_y = k_y
        self.k_theta = k_theta

        # Tolerâncias (Zonas Mortas)
        self.rho_tol = rho_tol
        self.theta_tol = theta_tol

        # Estados
        self.set_point = np.asarray(set_point, dtype=np.float64)

    def set_SP(self, set_point: np.ndarray):
        self.set_point = np.asarray(set_point, dtype=np.float64)

    @staticmethod
    @njit
    def _calc_errors(actual: np.ndarray, set_point: np.ndarray):
        """
        Cálculo puramente Cartesiano (Holonômico).
        Retorna os erros nos eixos globais e o erro de orientação normalizado.
        """
        dx = set_point[0] - actual[0]
        dy = set_point[1] - actual[1]
        
        # Normalização do ângulo para o robô não dar giros de 360°
        dtheta = set_point[2] - actual[2]
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        
        return dx, dy, dtheta

    def set_parameters(self, k_x: float, k_y: float, k_theta: float):
        self.k_x = k_x
        self.k_y = k_y
        self.k_theta = k_theta
        
    def set_tolerances(self, rho_tol: float, theta_tol: float):
        """Atualiza dinamicamente as margens de erro aceitáveis."""
        self.rho_tol = rho_tol
        self.theta_tol = theta_tol

    def get_control(self, actual_point: np.ndarray):
        """
        Calcula a ação de controle baseada no erro atual.
        Retorna a velocidade linear local (np.array) e a velocidade angular.
        """
        actual_point = np.asarray(actual_point, dtype=np.float64)
        dx, dy, dtheta = self._calc_errors(actual_point, self.set_point)

        # Distância Euclidiana (erro linear total)
        rho = np.sqrt(dx**2 + dy**2)

        # 1. Velocidades Globais baseadas no erro (Controle Proporcional Puro)
        vx_global = self.k_x * dx
        vy_global = self.k_y * dy
        w_target = self.k_theta * dtheta

        # 2. Rotação do Mundo para o Robô (Matriz de Rotação Inversa)
        theta = actual_point[2]
        c = np.cos(theta)
        s = np.sin(theta)

        vx_local = vx_global * c + vy_global * s
        vy_local = -vx_global * s + vy_global * c
        
        # 3. Zonas Mortas Independentes (Tratamento de Tolerância)
        # Se a distância linear estiver ok, paramos os motores de translação.
        if rho <= self.rho_tol:
            vx_local = 0.0
            vy_local = 0.0
            
        # Se a orientação estiver ok, paramos o giro.
        if abs(dtheta) <= self.theta_tol:
            w_target = 0.0

        return np.array([vx_local, vy_local]), w_target
    
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