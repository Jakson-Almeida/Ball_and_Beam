clc; clear; close all
syms s b3 b2 b1 b0 a0 real

K = (5/7)*9.81;
Ts=0.02;
Dp = s^2;
Np = K;

Nc = b3*s^3 + b2*s^2 + b1*s + b0;
Dc = (s^2 + 1^2)*(s + a0);

phi_cl = expand(Dp*Dc + Np*Nc);

p1 = -0.7*4 + 1i*4*sqrt(1-0.7^2);
p2 = conj(p1);
p3 = -16;
p4 = -16;
p5 = -20;

phi_d = expand((s-p1)*(s-p2)*(s-p3)*(s-p4)*(s-p5));

[clc,~] = coeffs(phi_cl,s,'All');
[dc, ~] = coeffs(phi_d, s,'All');

eqs = clc == dc;

sol = solve(eqs,[b3 b2 b1 b0 a0]);

C = tf(double([sol.b3 sol.b2 sol.b1 sol.b0]), ...
       conv([1 0 12],[1 double(sol.a0)]));

disp(C);

Cd = c2d(C, Ts, 't');

disp('Controlador discreto Cd(z):')
disp(Cd);

[numCd, denCd] = tfdata(Cd,'v');
[Ad, Bd, Cd_mat, Dd] = ssdata(Cd);