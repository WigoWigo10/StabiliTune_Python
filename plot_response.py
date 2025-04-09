import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

def plot_step_response_with_info(systems):
    """
    Plota a resposta ao degrau de uma ou mais funções de transferência de malha fechada,
    marcando tempo de acomodação, sobressinal (se houver) e valor final.
    Permite clicar nos pontos para exibir/ocultar tempo e amplitude (toggle).
    """
    fig, ax = plt.subplots()
    active_annotations = {}  # Dicionário de anotações ativas
    clickable_points = []

    # Primeira etapa: encontrar o maior tempo de acomodação
    settling_times = []
    for system, _ in systems:
        info = ctrl.step_info(system)
        settling_times.append(info['SettlingTime'])

    max_settling_time = max(max(settling_times), 5)  # Pelo menos 5 segundos
    max_time = max_settling_time * 1.80  # Folga
    max_amplitude = 0  # Inicializa para cálculo posterior

    def on_click(event):
        if event.inaxes != ax:
            return
        for x, y, label in clickable_points:
            if abs(event.xdata - x) < 0.05 * max_time and abs(event.ydata - y) < 0.05 * max_amplitude:
                key = (x, y, label)
                if key in active_annotations:
                    active_annotations[key].remove()
                    del active_annotations[key]
                else:
                    if "Valor Final" in label:
                        text = f'{label}\nSaída: {y:.4f}'
                    else:
                        text = f'{label}\nTempo: {x:.4f}s\nSaída: {y:.4f}'

                    annotation = ax.annotate(text,
                                            xy=(x, y), xytext=(20, 20),
                                            textcoords='offset points', fontsize=9,
                                            bbox=dict(boxstyle="round", fc="w"),
                                            arrowprops=dict(arrowstyle="->"))
                    active_annotations[key] = annotation
                plt.draw()
                break

    for system, label in systems:
        info = ctrl.step_info(system)
        settling_time = info['SettlingTime']
        y_max = info['Peak']
        y_final = info['SteadyStateValue']
        overshoot = ((y_max - y_final) / y_final) * 100
        final_time = max_time  # todos os sistemas compartilham o mesmo tempo de simulação

        t = np.linspace(0, final_time, 1000)
        Tout, y = ctrl.step_response(system, t)

        max_amplitude = max(max_amplitude, y_max * 1.20)

        ax.plot(Tout, y, label=label)
        ax.axhline(y=y_max, linestyle='--', color='gray')

        # Tempo de acomodação
        idx_settle = np.where(t >= settling_time)[0][0]
        ax.scatter(settling_time, y[idx_settle], color='red', zorder=5)
        clickable_points.append((settling_time, y[idx_settle], f"{label} - Tempo de Acomodação"))

        # Sobressinal
        if overshoot > 0:
            overshoot_time = Tout[np.argmax(y)]
            ax.scatter(overshoot_time, y_max, color='green', zorder=5)
            clickable_points.append((overshoot_time, y_max, f"{label} - Sobressinal Máximo"))

        # Valor final
        final_x = final_time * 0.999
        ax.scatter(final_x, y_final, color='blue', zorder=5)
        clickable_points.append((final_x, y_final, f"{label} - Valor Final"))


    ax.set_title("Resposta ao Degrau dos Sistemas")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Saída")
    ax.grid(True)
    ax.legend()
    ax.set_xlim(0, max_time)
    ax.set_ylim(0, max_amplitude)
    fig.canvas.mpl_connect('button_press_event', on_click)
    plt.show()
