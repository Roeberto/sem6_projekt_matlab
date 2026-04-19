% =========================================================================
%  run_projekt_lqr.m
%  Uruchamia model Simulinka projekt_lqr_ciagly.slx, pobiera sygnały
%  zalogowane blokami To Workspace (h, Q_in, Q_out) i rysuje wykresy.
% =========================================================================
clear; clc; close all;

%% ── 1. WARTOŚCI POCZĄTKOWE / PARAMETRY ───────────────────────────────────
g    = 9.81;
r    = 1.0;       A    = pi*r^2;
H    = 2.0;
r_v  = 0.04;      A_v  = pi*r_v^2;
mu_v = 0.97;

h_ref   = 1.0;                              % poziom zadany [m]
h0      = 0.4;                              % warunek początkowy [m]
Q_in_0  = mu_v*A_v*sqrt(2*g*h_ref);         % przepływ punktu pracy

% Linearyzacja
A_lin = -mu_v*A_v*sqrt(g/(2*h_ref))/A;
B_lin =  1/A;

% LQR
Q_lqr = 100;   R_lqr = 1;
[K_lqr,~,pol_cl] = lqr(A_lin,B_lin,Q_lqr,R_lqr);

% Pompa i czas symulacji
Q_pump = 2*Q_in_0;
T_sim  = 180;                               % [s]

fprintf('=== PARAMETRY ===\n');
fprintf('  h_ref  = %.3f m,  h0 = %.3f m\n',h_ref,h0);
fprintf('  Q_in_0 = %.5f m^3/s\n',Q_in_0);
fprintf('  K_LQR  = %.4f,  biegun zam. petli = %.3f 1/s\n',K_lqr,pol_cl);
fprintf('  T_sim  = %d s\n\n',T_sim);

%% ── 2. PRZYGOTOWANIE MODELU ──────────────────────────────────────────────
mdl = 'projekt_lqr_ciagly';
if ~exist([mdl '.slx'],'file')
    fprintf('Model %s.slx nie istnieje - buduję...\n',mdl);
    run('build_simulink_lqr.m');
    close_system(mdl,0);
end
load_system(mdl);
set_param(mdl,'StopTime',num2str(T_sim));

%% ── 3. SYMULACJA ─────────────────────────────────────────────────────────
fprintf('Uruchamiam symulację...\n');
simOut = sim(mdl,'ReturnWorkspaceOutputs','on');

%% ── 4. POBRANIE SYGNAŁÓW ─────────────────────────────────────────────────
h_ts    = simOut.get('h_ts');
Qin_ts  = simOut.get('Qin_ts');
Qout_ts = simOut.get('Qout_ts');

t      = h_ts.Time;
h      = h_ts.Data;
Q_in   = Qin_ts.Data;
Q_out  = Qout_ts.Data;
err    = h_ref - h;

%% ── 5. STATYSTYKI ────────────────────────────────────────────────────────
fprintf('\n=== STATYSTYKI ===\n');
fprintf('  Średni poziom  : %.4f m  (zadany %.4f m)\n',mean(h),h_ref);
fprintf('  RMSE           : %.4f m\n',sqrt(mean(err.^2)));
fprintf('  Min/Max h      : %.4f / %.4f m\n',min(h),max(h));
fprintf('  Śr. Q_in       : %.5f m^3/s (punkt pracy %.5f)\n',...
        mean(Q_in),Q_in_0);
[~,idx95] = min(abs(h-0.95*h_ref));
fprintf('  t_0.95*h_ref   : %.2f s\n',t(idx95));

%% ── 6. WYKRESY ───────────────────────────────────────────────────────────
figure('Name','Symulacja Simulink – LQR ciągły','Position',[80 60 1100 820]);

subplot(3,1,1);
plot(t,h,'b-','LineWidth',1.6); hold on;
plot(t,h_ref*ones(size(t)),'r--','LineWidth',1.3);
plot([0 t(end)],[h0 h0],'k:','LineWidth',0.8);
ylabel('h [m]'); ylim([0 H+0.1]); grid on;
title(sprintf('Poziom cieczy (K_{LQR} = %.3f, h_0 = %.2f m)',K_lqr,h0));
legend('h(t)','h_{ref}','h_0','Location','best');

subplot(3,1,2);
plot(t,Q_in*1000, 'g-','LineWidth',1.6); hold on;
plot(t,Q_out*1000,'r-','LineWidth',1.3);
plot([0 t(end)],[Q_in_0 Q_in_0]*1000,'k--','LineWidth',0.8);
ylabel('Q [l/s]'); grid on;
title('Dopływ Q_{in} (sterowanie LQR) i odpływ Q_{out}');
legend('Q_{in}','Q_{out}','Q_{in,0}','Location','best');

subplot(3,1,3);
plot(t,err,'m-','LineWidth',1.5); hold on;
plot([0 t(end)],[0 0],'k--');
xlabel('Czas [s]'); ylabel('e = h_{ref} - h  [m]'); grid on;
title('Uchyb regulacji');

sgtitle('Model ciągły LQR – wyniki z Simulinka',...
        'FontSize',14,'FontWeight','bold');

%% ── 7. PŁASZCZYZNA FAZOWA (opcjonalnie) ──────────────────────────────────
figure('Name','Płaszczyzna fazowa','Position',[1200 60 500 400]);
plot(h,Q_in*1000,'b-','LineWidth',1.4); hold on;
plot(h_ref,Q_in_0*1000,'ro','MarkerSize',10,'LineWidth',2);
plot(h0,Q_in(1)*1000,'ks','MarkerSize',10,'LineWidth',2);
xlabel('h [m]'); ylabel('Q_{in} [l/s]'); grid on;
title('Trajektoria w przestrzeni (h, Q_{in})');
legend('trajektoria','punkt pracy','start','Location','best');

fprintf('\nGotowe. Zamknij okna wykresów, aby zakończyć.\n');
