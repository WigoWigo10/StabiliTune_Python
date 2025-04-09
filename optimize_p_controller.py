import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.optimize import minimize
from scipy.optimize import differential_evolution  # Para algoritmos genéticos (se preferir)
from plot_response import plot_step_response_with_info

def optimize_p_controller(G_open_or_closed, settling_time_spec, is_closed=False):
    """
    Otimiza um controlador proporcional (P) para atender à especificação de tempo de acomodação.
    Se for fornecida a malha aberta, o sistema será fechado automaticamente com feedback unitário.

    Parâmetros:
    -----------
    G_open_or_closed : control.TransferFunction
        Sistema fornecido (malha aberta ou fechada).
    settling_time_spec : float
        Tempo de acomodação desejado.
    is_closed : bool
        Indica se G já é a função de transferência de malha fechada.

    Retorna:
    --------
    H_controlled : control.TransferFunction
        Sistema com o controlador P aplicado.
    """
    if not is_closed:
        G_open = G_open_or_closed
        G_original = ctrl.feedback(G_open, 1)
    else:
        G_original = G_open_or_closed
        G_open = G_original / (1 - G_original)  # aproximação para obter malha aberta

    # Diagnóstico de estabilidade (análise do diagrama de Bode)
    mag, phase, omega = ctrl.bode(G_original, dB=True, plot=False)
    if np.any(mag > 0):
        print("Aviso: o sistema original é instável. Tentando estabilizá-lo.")

    # Função de custo considerando estabilidade e tempo de acomodação
    def cost(Kp):
        if Kp <= 0:
            return 1e6

        Kp = float(Kp)
        Gc = ctrl.TransferFunction([Kp], [1])
        H_cl = ctrl.feedback(ctrl.series(Gc, G_open), 1)

        # Verificar se a resposta ao degrau existe e não está vazia
        try:
            Tout, y = ctrl.step_response(H_cl)
            if len(y) == 0:
                raise ValueError("Resposta ao degrau vazia.")
            info = ctrl.step_info(H_cl)
        except Exception as e:
            print(f"Erro ao calcular informações de resposta ao degrau: {e}")
            return 1e6  # Penaliza sistemas com erro de resposta

        # Penalizar sistemas instáveis
        if any(p.real >= 0 for p in ctrl.poles(H_cl)):
            return 1e6

        return abs(info['SettlingTime'] - settling_time_spec)

    # Tentativas de otimização com várias interações
    best_Kp = None
    best_cost = float('inf')
    attempts = 0
    max_attempts = 20  # Número máximo de tentativas

    while attempts < max_attempts:
        print(f"Tentativa #{attempts + 1}")
        result = minimize(cost, x0=[1.0], method='L-BFGS-B', bounds=[(0.001, 1000)])

        if not result.success:
            print(f"O L-BFGS-B não convergiu. Tentando com algoritmo genético...")
            result = differential_evolution(cost, bounds=[(0.001, 1000)], strategy='best1bin', maxiter=1000)

        Kp = result.x[0]

        if result.success and cost(Kp) < best_cost:
            best_Kp = Kp
            best_cost = cost(Kp)
            print(f"Kp ótimo encontrado: {best_Kp:.4f} com custo {best_cost:.4f}")
            break

        attempts += 1

    if best_Kp is None:
        print(f"A otimização não convergiu após {max_attempts} tentativas.")
        return None

    # Gerando o sistema com o controlador P
    Gc_opt = ctrl.TransferFunction([best_Kp], [1])
    G_controlled = ctrl.series(Gc_opt, G_open)
    H_controlled = ctrl.feedback(G_controlled, 1)

    # Gráficos de comparação
    plot_step_response_with_info([
        (G_original, "Sistema Original"),
        (H_controlled, f"Com Controlador P (Kp = {best_Kp:.3f})")
    ])

    return H_controlled
