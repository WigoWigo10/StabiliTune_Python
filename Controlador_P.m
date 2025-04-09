% Definindo o sistema original (planta)
num_G = [0.1];  % Numerador da função de transferência
den_G = [2 ,1]; % Denominador da função de transferência

G = tf(num_G, den_G); % Criando a função de transferência da planta
G

% sisotool(G);

% Especificações de desempenho: 
% Erro em regime permanente nulo, Sobressinal <= 5%, Tempo de acomodação <= 2 segundos

Sistema_original = feedback(G, 1);  % Fechando a malha com unidade de feedback
Sistema_original

ControladorP = 1;

Sistema_com_Controlador = feedback(ControladorP*G, 1);  % Fechando a malha com unidade de feedback
Sistema_com_Controlador

% Comparando a resposta ao Degrau sem e com compensador
figure;
% subplot(2,1,1);
step(Sistema_original);
title('Resposta do Sistema não Compensado');
xlabel('Tempo (s)');
ylabel('Saída');
grid on;

% subplot(2,1,2);
% step(Sistema_com_Controlador);
% title('Resposta do Sistema com Compensador de Atraso em Fase');
% xlabel('Tempo (s)');
% ylabel('Saída');
% grid on;
