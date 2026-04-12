% =========================================================================
% ZBIORNIK – LQR jako przełącznik pompy ON/OFF
% Pompa OFF → Q_in = 0
% Pompa ON  → Q_in = Q_pump (stały przepływ)
% LQR decyduje kiedy włączyć/wyłączyć pompę
% =========================================================================
clear; clc; close all;

%% ── 1. PARAMETRY FIZYCZNE ────────────────────────────────────────────────
g    = 9.81;
r    = 1.0;
A    = pi * r^2;
H    = 2.0;

r_v  = 0.04;
A_v  = pi * r_v^2;
mu_v = 0.97;

%% ── 2. PUNKT PRACY ───────────────────────────────────────────────────────
h_ref   = 1.0;
Q_out_0 = mu_v * A_v * sqrt(2*g*h_ref);
Q_in_0  = Q_out_0;

fprintf('=== PUNKT PRACY ===\n');
fprintf('  h_ref  = %.4f m\n',   h_ref);
fprintf('  Q_in_0 = %.6f m^3/s\n\n', Q_in_0);

%% ── 3. LINEARYZACJA ──────────────────────────────────────────────────────
A_lin = -mu_v * A_v * sqrt(g / (2*h_ref)) / A;
B_lin =  1.0 / A;
C_lin =  1;
D_lin =  0;

fprintf('=== MODEL LINIOWY ===\n');
fprintf('  A_lin = %.6f 1/s  (T_ol = %.1f s)\n\n', A_lin, -1/A_lin);

%% ── 4. PROJEKT LQR ───────────────────────────────────────────────────────
Q_lqr = 100;
R_lqr = 1;

[K_lqr, ~, poles_cl] = lqr(A_lin, B_lin, Q_lqr, R_lqr);

fprintf('=== REGULATOR LQR ===\n');
fprintf('  K_lqr   = %.4f\n', K_lqr);
fprintf('  Tau_cl  = %.2f s\n\n', -1/poles_cl);

%% ── 5. PARAMETRY POMPY ───────────────────────────────────────────────────
Q_pump      = 2.0 * Q_in_0;   % stały przepływ gdy pompa ON [m^3/s]
                              % > Q_in_0 żeby zbiornik mógł się napełniać
t_min_pump  = 5;              % minimalny czas ciągłej pracy pompy [s]

% Próg LQR: pompa włącza się gdy LQR żąda przepływu > Q_pump/2
% (tzn. gdy poziom wyraźnie poniżej zadanego)
Q_threshold = Q_pump / 2;

fprintf('=== POMPA ===\n');
fprintf('  Q_pump     = %.6f m^3/s\n', Q_pump);
fprintf('  t_min_pump = %d s\n', t_min_pump);
fprintf('  Q_threshold= %.6f m^3/s\n\n', Q_threshold);

%% ── 6. SYMULACJA ─────────────────────────────────────────────────────────
dt    = 1.0;
T_sim = 180;
t_vec = 0:dt:T_sim;
N     = length(t_vec);

h_sim      = zeros(1, N);
Q_in_sim   = zeros(1, N);
Q_out_sim  = zeros(1, N);
d_sim      = zeros(1, N);
pump_state = zeros(1, N);

h_sim(1) = 0.4;      % warunek początkowy

d_min  = 0.000;
d_max  = 0.010;
T_dist = 5;

pump_on      = false;
t_pump_start = 0;

rng(42);

for k = 1:N-1

    % ── Losowe zakłócenie ─────────────────────────────────────────────────
    if mod(k, round(T_dist/dt)) == 0 || k == 1
        d_sim(k) = d_min + (d_max - d_min)*rand();
    else
        d_sim(k) = d_sim(k-1);
    end

    % ── Sygnał LQR → żądany przepływ ciągły ──────────────────────────────
    dh_err = h_sim(k) - h_ref;
    Q_des  = Q_in_0 - K_lqr * dh_err;   % ciągły sygnał LQR

    % ── Logika ON/OFF pompy ───────────────────────────────────────────────
    lqr_chce_pompe = Q_des > Q_threshold;

    if ~pump_on && lqr_chce_pompe
        % Poziom za niski → włącz pompę
        pump_on      = true;
        t_pump_start = t_vec(k);

    elseif pump_on && ~lqr_chce_pompe
        % LQR chce wyłączyć → sprawdź minimalny czas
        if (t_vec(k) - t_pump_start) >= t_min_pump
            pump_on = false;
        end
        % jeśli nie minął t_min_pump → pompa dalej ON
    end

    % ── Rzeczywisty dopływ: stały Q_pump lub 0 ───────────────────────────
    if pump_on
        Q_in_sim(k) = Q_pump;   % STAŁY przepływ
    else
        Q_in_sim(k) = 0;        % BRAK dopływu
    end
    pump_state(k) = double(pump_on);

    % ── Równanie stanu (Euler) ────────────────────────────────────────────
    h_safe        = max(h_sim(k), 0.001);
    Q_valve       = mu_v * A_v * sqrt(2*g*h_safe);
    Q_out_sim(k)  = Q_valve + d_sim(k);

    h_sim(k+1) = h_sim(k) + dt * (Q_in_sim(k) - Q_out_sim(k)) / A;
    h_sim(k+1) = max(0.001, min(H, h_sim(k+1)));
end

d_sim(N)     = d_sim(N-1);
Q_in_sim(N)  = Q_in_sim(N-1);
Q_out_sim(N) = Q_out_sim(N-1);
pump_state(N)= pump_state(N-1);

fprintf('=== STATYSTYKI ===\n');
fprintf('  Średni poziom    : %.4f m  (zadany: %.4f m)\n', mean(h_sim), h_ref);
fprintf('  Odch. std.       : %.4f m\n', std(h_sim));
fprintf('  RMSE             : %.4f m\n', sqrt(mean((h_sim - h_ref).^2)));
fprintf('  Min/Max h        : %.4f / %.4f m\n', min(h_sim), max(h_sim));
fprintf('  Czas pracy pompy : %.1f min / %.1f min (%.1f%%)\n', ...
        sum(pump_state)*dt/60, T_sim/60, 100*mean(pump_state));

%% ── 7. WYKRESY ───────────────────────────────────────────────────────────
t_ax = t_vec / 60;

figure('Name','Zbiornik – LQR ON/OFF', 'Position',[100 100 1100 900]);

subplot(4,1,1);
plot(t_ax, h_sim, 'b-', 'LineWidth', 1.5); hold on;
plot(t_ax, h_ref*ones(size(t_ax)), 'r--', 'LineWidth', 2);
ylabel('h [m]'); ylim([0, H+0.1]);
title(sprintf('Poziom cieczy (K_{LQR}=%.2f, Q_{pump}=%.4f m^3/s, t_{min}=%ds)', ...
      K_lqr, Q_pump, t_min_pump));
legend('h(t)', 'h_{ref} = 1 m', 'Location','best');
grid on;

subplot(4,1,2);
stairs(t_ax, Q_in_sim*1000, 'g-', 'LineWidth', 2); hold on;
plot(t_ax, Q_out_sim*1000, 'r-', 'LineWidth', 1.2);
ylabel('Q [l/s]');
title('Dopływ Q_{in} (schodkowy: 0 lub Q_{pump}) i odpływ Q_{out}');
legend('Q_{in} – pompa', 'Q_{out} – odpływ', 'Location','best');
grid on;

subplot(4,1,3);
plot(t_ax, d_sim*1000, 'm-', 'LineWidth', 1.5);
ylabel('d [l/s]');
title('Losowe zakłócenie – dodatkowy odpływ');
grid on;

subplot(4,1,4);
area(t_ax, pump_state, 'FaceColor',[0.2 0.7 0.3], 'FaceAlpha', 0.6);
xlabel('Czas [min]'); ylabel('Stan [0/1]');
title(sprintf('Stan pompy ON/OFF  (t_{min} = %d s)', t_min_pump));
ylim([-0.1, 1.3]); grid on;

sgtitle('Zbiornik walcowy – LQR jako przełącznik pompy ON/OFF', ...
        'FontSize', 14, 'FontWeight', 'bold');