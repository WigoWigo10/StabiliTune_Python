import control as ctrl
from optimize_p_controller import optimize_p_controller
from plot_response import plot_step_response_with_info

# Sistema de malha aberta
G = ctrl.TransferFunction([1], [1, -2])

# Especificação do tempo de acomodação desejado (em segundos)
tempo_desejado = 2.0

# Otimiza o controlador P e obtém a malha fechada com o controle
H_controlado = optimize_p_controller(G, tempo_desejado, is_closed=False)

# Mostra a resposta detalhada da malha controlada
plot_step_response_with_info(H_controlado)
