clc; clear; close all

%% =========================================================
% PARÂMETROS
%% =========================================================
g  = 9.81;
K  = (5/7)*g;
Ts = 0.02;

Tf = 10;
N  = Tf/Ts;
t  = (0:N-1)*Ts;

%% =========================================================
% REFERÊNCIA E DISTÚRBIO
%% =========================================================
r = 0.1*ones(1,N);           % referência degrau
d = zeros(1,N);
d(t > 5) = 0.02;             % distúrbio aplicado em t = 5 s

%% =========================================================
% MODELO LINEAR CONTÍNUO
%% =========================================================
A = [0 1;
     0 0];
B = [0;
     K];
C = [1 0];
D = 0;

sys_c = ss(A,B,C,D);
sys_d = c2d(sys_c, Ts, 'zoh');
[Phi, Gamma, ~, ~] = ssdata(sys_d);

%% =========================================================
% SISTEMA AUMENTADO (AÇÃO INTEGRAL)
%% =========================================================
Phi_a = [ Phi        zeros(2,1);
         -Ts*C       1          ];

Gamma_a = [ Gamma;
             0     ];

Gamma_r = [ 0;
            0;
            Ts ];

%% =========================================================
% GANHOS DO CONTROLADOR (PROJETO)
%% =========================================================
Lk1 = 4.7046;
Lk2 = 1.6662;
Lk3 = -6.4214;
L   = [Lk1 Lk2 Lk3];

%% =========================================================
% SIMULAÇÃO NÃO LINEAR
%% =========================================================
xa = zeros(3,N);
y  = zeros(1,N);
u  = zeros(1,N);

for k = 1:N-1
    % Controle com ação integral
    u(k) = -L * xa(:,k);
    u(k) = max(min(u(k),0.4),-0.4);   % saturação física
    
    % Estados
    x1 = xa(1,k);
    x2 = xa(2,k);
    
    % Dinâmica NÃO LINEAR
    x1_dot = x2;
    x2_dot = K*sin(u(k)) + d(k);      % distúrbio
    xI_dot = r(k) - x1;
    
    % Integração Euler
    xa(1,k+1) = x1 + Ts*x1_dot;
    xa(2,k+1) = x2 + Ts*x2_dot;
    xa(3,k+1) = xa(3,k) + Ts*xI_dot;
    
    y(k) = x1;
end
y(N) = xa(1,N);

%% =========================================================
% SIMULAÇÃO LINEAR (COMPARAÇÃO)
%% =========================================================
xaL = zeros(3,N);
yL  = zeros(1,N);
uL  = zeros(1,N);

for k = 1:N-1
    uL(k) = -L * xaL(:,k);
    xaL(:,k+1) = Phi_a*xaL(:,k) + Gamma_a*uL(k) + Gamma_r*r(k);
    yL(k) = C*xaL(1:2,k);
end
yL(N) = C*xaL(1:2,N);

%% =========================================================
% GRÁFICOS – RESPOSTA TEMPORAL
%% =========================================================
figure
plot(t,r,'k--','LineWidth',1.5); hold on
plot(t,yL,'b','LineWidth',2)
plot(t,y,'r','LineWidth',2)
grid on
xlabel('Tempo (s)')
ylabel('Posição da bola (m)')
legend('Referência','Linear','Não linear')
title('Rastreamento de Referência – Comparação Linear × Não Linear')

%% =========================================================
% GRÁFICO DO CONTROLE
%% =========================================================
figure
plot(t,uL,'b','LineWidth',1.8); hold on
plot(t,u,'r','LineWidth',1.8)
grid on
xlabel('Tempo (s)')
ylabel('Ângulo da viga (rad)')
legend('Linear','Não linear')
title('Sinal de Controle com Saturação')

%% =========================================================
% RESPOSTA A DISTÚRBIO
%% =========================================================
figure
plot(t,d,'k--','LineWidth',1.5); hold on
plot(t,y,'r','LineWidth',2)
grid on
xlabel('Tempo (s)')
ylabel('Saída')
legend('Distúrbio','Saída')
title('Resposta do Sistema Não Linear a Distúrbio')

%% =========================================================
% FUNÇÕES DE SENSIBILIDADE (MODELO LINEAR)
%% =========================================================
Acl = Phi - Gamma*[Lk1 Lk2];
sys_cl = ss(Acl, Gamma, C, 0, Ts);

S = feedback(1,sys_cl);      % sensibilidade
T = feedback(sys_cl,1);      % complementar

figure
bodemag(S,T)
grid on
legend('S(z)','T(z)')
title('Funções de Sensibilidade')

%% =========================================================
% RESPOSTA EM FREQUÊNCIA
%% =========================================================
figure
bode(sys_cl)
grid on
title('Resposta em Frequência – Sistema em Malha Fechada')





disturbios = [0.05 0.1 0.15 0.2];
cores = lines(length(disturbios));

figure; hold on
for i = 1:length(disturbios)

    d = zeros(1,N);
    d(t > 5) = disturbios(i);   % distúrbio aplicado em t = 5s

    xa_d = zeros(3,N);
    y_d  = zeros(1,N);

    for k = 1:N-1
        u_k = -L * xa_d(:,k);
        u_k = max(min(u_k,0.4),-0.4);

        x1 = xa_d(1,k);
        x2 = xa_d(2,k);

        x1_dot = x2;
        x2_dot = K*sin(u_k) + d(k);
        xI_dot = r(k) - x1;

        xa_d(1,k+1) = x1 + Ts*x1_dot;
        xa_d(2,k+1) = x2 + Ts*x2_dot;
        xa_d(3,k+1) = xa_d(3,k) + Ts*xI_dot;

        y_d(k) = x1;
    end
    y_d(N) = xa_d(1,N);

    plot(t,y_d,'LineWidth',2,'Color',cores(i,:))
end

grid on
xlabel('Tempo (s)')
ylabel('Posição da bola (m)')
legend('d = 0.05','d = 0.10','d = 0.15','d = 0.20','Location','Best')
title('Resposta do Sistema Não Linear a Diferentes Distúrbios')




referencias = [0.1 0.2 0.5 0.7];
cores = lines(length(referencias));

figure; hold on
for i = 1:length(referencias)

    r_var = referencias(i)*ones(1,N);

    xa_r = zeros(3,N);
    y_r  = zeros(1,N);

    for k = 1:N-1
        u_k = -L * xa_r(:,k);
        u_k = max(min(u_k,0.4),-0.4);

        x1 = xa_r(1,k);
        x2 = xa_r(2,k);

        x1_dot = x2;
        x2_dot = K*sin(u_k);
        xI_dot = r_var(k) - x1;

        xa_r(1,k+1) = x1 + Ts*x1_dot;
        xa_r(2,k+1) = x2 + Ts*x2_dot;
        xa_r(3,k+1) = xa_r(3,k) + Ts*xI_dot;

        y_r(k) = x1;
    end
    y_r(N) = xa_r(1,N);

    plot(t,y_r,'LineWidth',2,'Color',cores(i,:))
end

grid on
xlabel('Tempo (s)')
ylabel('Posição da bola (m)')
legend('r = 0.1','r = 0.2','r = 0.5','r = 0.7','Location','Best')
title('Resposta do Sistema Não Linear a Diferentes Referências')




figure; hold on
for i = 1:length(referencias)

    r_var = referencias(i)*ones(1,N);

    xa_u = zeros(3,N);
    u_u  = zeros(1,N);

    for k = 1:N-1
        u_u(k) = -L * xa_u(:,k);
        u_u(k) = max(min(u_u(k),0.4),-0.4);

        x1 = xa_u(1,k);
        x2 = xa_u(2,k);

        x1_dot = x2;
        x2_dot = K*sin(u_u(k));
        xI_dot = r_var(k) - x1;

        xa_u(1,k+1) = x1 + Ts*x1_dot;
        xa_u(2,k+1) = x2 + Ts*x2_dot;
        xa_u(3,k+1) = xa_u(3,k) + Ts*xI_dot;
    end

    plot(t,u_u,'LineWidth',1.8,'Color',cores(i,:))
end

grid on
xlabel('Tempo (s)')
ylabel('Ângulo da viga (rad)')
legend('r = 0.1','r = 0.2','r = 0.5','r = 0.7','Location','Best')
title('Ação de Controle para Diferentes Referências')