import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.optimize import minimize
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

    # Verificar se o sistema original é instável
    poles = ctrl.pole(G_original)
    if any(p.real >= 0 for p in poles):
        print("Aviso: o sistema original é instável. O controlador tentará estabilizá-lo.")

    def cost(Kp):
        if Kp <= 0:
            return 1e6
        Kp = float(Kp)
        Gc = ctrl.TransferFunction([Kp], [1])
        H_cl = ctrl.feedback(ctrl.series(Gc, G_open), 1)
        info = ctrl.step_info(H_cl)

        # Penalizar sistemas instáveis
        if any(p.real >= 0 for p in ctrl.pole(H_cl)):
            return 1e6

        return abs(info['SettlingTime'] - settling_time_spec)

    result = minimize(cost, x0=[1.0], method='L-BFGS-B', bounds=[(0.001, 1000)])
    Kp = result.x[0]

    if not result.success:
        print(f"Atenção: Otimização não convergiu. Motivo: {result.message}")

    # Gerando o sistema com o controlador P
    Gc_opt = ctrl.TransferFunction([Kp], [1])
    G_controlled = ctrl.series(Gc_opt, G_open)
    H_controlled = ctrl.feedback(G_controlled, 1)

    # Gráficos de comparação
    plot_step_response_with_info([
        (G_original, "Sistema Original"),
        (H_controlled, f"Com Controlador P (Kp = {Kp:.3f})")
    ])

    print(f"Kp ótimo encontrado: {Kp:.4f}")
    return H_controlled
