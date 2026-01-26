clc; clear;

%% =========================
% Parâmetros
%% =========================
K  = (5/7)*9.81;
Ts = 0.02;

Tf = 10;                     % tempo total de simulação (s)
N  = Tf/Ts;                  % número de amostras
t  = (0:N-1)*Ts;

%% =========================
% Modelo contínuo (Ball and Beam)
%% =========================
A = [0 1;
     0 0];

B = [0;
     K];

C = [1 0];
D = 0;

sys_c = ss(A,B,C,D);

%% =========================
% Discretização
%% =========================
sys_d = c2d(sys_c, Ts, 'zoh');
[Phi, Gamma, ~, ~] = ssdata(sys_d);

%% =========================
% Sistema aumentado (integrador do erro)
%% =========================
Phi_a = [ Phi        zeros(2,1);
         -Ts*C       1          ];

Gamma_a = [ Gamma;
             0     ];

Gamma_r = [ 0;
            0;
            Ts ];          % entrada da referência

%% =========================
% Polos desejados
%% =========================
zeta = 0.7;
wn   = 2.3;

p1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
p2 = conj(p1);
p3 = 6*real(p1);            % polo do integrador

z_poles = exp([p1 p2 p3]*Ts);

%% =========================
% Ganho por Ackermann
%% =========================
L_a = acker(Phi_a, Gamma_a, z_poles);
Lk1 = L_a(1);
Lk2 = L_a(2);
Lk3 = L_a(3);

disp('Ganhos:')
disp(['L1  = ', num2str(Lk1)])
disp(['L2= ', num2str(Lk2)])
disp(['L3 = ', num2str(Lk3)])

%% =========================
% Simulação
%% =========================
xa = zeros(3,N);            % estados aumentados
y  = zeros(1,N);            % saída
u  = zeros(1,N);            % controle

r = 0.1*ones(1,N);              % referência degrau unitário

for k = 1:N-1
    u(k) = -L_a * xa(:,k);
    xa(:,k+1) = Phi_a*xa(:,k) + Gamma_a*u(k) + Gamma_r*r(k);
    y(k) = C*xa(1:2,k);
end
y(N) = C*xa(1:2,N);

%% =========================
% Gráficos
%% =========================
figure
plot(t, r, 'k--', 'LineWidth',1.5); hold on
plot(t, y, 'b', 'LineWidth',2)
grid on
xlabel('Tempo (s)')
ylabel('Posição da bola')
legend('Referência','Saída')
title('Rastreamento de referência degrau')

figure
plot(t, u, 'r', 'LineWidth',1.5)
grid on
xlabel('Tempo (s)')
ylabel('Controle u(k)')
title('Sinal de controle')