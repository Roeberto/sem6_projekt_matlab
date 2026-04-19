% =========================================================================
%  build_simulink_lqr.m
%  Programowo tworzy CIĄGŁY model Simulinka: zbiornik + pompa + LQR
%  Wzory zgodne z sem6_projekt_v1.m (linearyzacja Torricellego + ARE)
% =========================================================================
clear; clc; close all;

%% ── 1. PARAMETRY FIZYCZNE ────────────────────────────────────────────────
g    = 9.81;
r    = 1.0;        A   = pi*r^2;
H    = 2.0;
r_v  = 0.04;       A_v = pi*r_v^2;
mu_v = 0.97;

%% ── 2. PUNKT PRACY ───────────────────────────────────────────────────────
h_ref  = 1.0;
Q_in_0 = mu_v*A_v*sqrt(2*g*h_ref);

%% ── 3. LINEARYZACJA (A, B ciągłe) ────────────────────────────────────────
A_lin = -mu_v*A_v*sqrt(g/(2*h_ref))/A;     % a = df/dh
B_lin =  1/A;                              % b = df/dQin

%% ── 4. LQR (ARE skalarny) ────────────────────────────────────────────────
Q_lqr = 100;   R_lqr = 1;
[K_lqr,~,poles_cl] = lqr(A_lin,B_lin,Q_lqr,R_lqr);

%% ── 5. POMPA (saturacja fizyczna) ────────────────────────────────────────
Q_pump = 2*Q_in_0;
h0     = 0.4;        % warunek początkowy poziomu

fprintf('K_LQR = %.4f, a = %.5f, b = %.5f, pol = %.3f\n',...
        K_lqr,A_lin,B_lin,poles_cl);

%% ── 6. BUDOWA MODELU SIMULINKA ───────────────────────────────────────────
mdl = 'projekt_lqr_ciagly';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

set_param(mdl,'Solver','ode45','StopTime','180',...
              'SolverType','Variable-step');

add = @(type,name,pos,varargin) add_block(type,[mdl '/' name],...
        'Position',pos,varargin{:});

% --- referencja h_ref -----------------------------------------------------
add('simulink/Sources/Constant','h_ref',[30 40 70 70],...
    'Value','h_ref');

% --- sumator błędu e = h_ref - h -----------------------------------------
add('simulink/Math Operations/Sum','e',[120 40 150 70],...
    'Inputs','+-','IconShape','round');

% --- wzmocnienie LQR ------------------------------------------------------
add('simulink/Math Operations/Gain','K_LQR',[190 35 240 75],...
    'Gain','K_lqr');

% --- offset Q_in_0 (dodajemy, bo u = Q_in_0 - K*(h - h_ref)) -------------
add('simulink/Math Operations/Sum','offset',[280 40 310 70],...
    'Inputs','++','IconShape','round');

add('simulink/Sources/Constant','Q0',[190 100 240 130],...
    'Value','Q_in_0');

% --- saturacja pompy 0..Q_pump -------------------------------------------
add('simulink/Discontinuities/Saturation','SatPompy',[340 35 390 75],...
    'UpperLimit','Q_pump','LowerLimit','0');

% --- zakłócenie d (domyślnie 0, user może podmienić) ---------------------
add('simulink/Sources/Constant','d',[340 150 390 180],'Value','0');

% --- sumator bilansu: Q_in - Q_out - d -----------------------------------
add('simulink/Math Operations/Sum','bilans',[440 55 480 110],...
    'Inputs','+--','IconShape','round');

% --- 1/A -----------------------------------------------------------------
add('simulink/Math Operations/Gain','invA',[510 65 560 95],...
    'Gain','1/A');

% --- Integrator (h) ------------------------------------------------------
add('simulink/Continuous/Integrator','Int_h',[590 60 630 100],...
    'InitialCondition','h0','LowerSaturationLimit','0.001',...
    'UpperSaturationLimit','H','LimitOutput','on');

% --- Q_out = mu_v*A_v*sqrt(2*g*h) ----------------------------------------
add('simulink/Math Operations/Gain','g2',[440 170 490 200],...
    'Gain','2*g');
add('simulink/Math Operations/Sqrt','sqrtH',[510 170 550 200]);
add('simulink/Math Operations/Gain','muAv',[570 170 630 200],...
    'Gain','mu_v*A_v');

% --- wyjście Scope -------------------------------------------------------
add('simulink/Sinks/Scope','Scope_h',[680 55 720 105],...
    'NumInputPorts','2');

add('simulink/Sinks/Scope','Scope_Q',[440 5 480 35]);
add('simulink/Sources/Constant','h_ref_vis',[620 10 660 40],...
    'Value','h_ref');

% --- To Workspace: h, Q_in, Q_out (format Timeseries) --------------------
add('simulink/Sinks/To Workspace','Log_h',[680 130 730 160],...
    'VariableName','h_ts','SaveFormat','Timeseries');
add('simulink/Sinks/To Workspace','Log_Qin',[400 250 450 280],...
    'VariableName','Qin_ts','SaveFormat','Timeseries');
add('simulink/Sinks/To Workspace','Log_Qout',[680 170 730 200],...
    'VariableName','Qout_ts','SaveFormat','Timeseries');

%% ── 7. POŁĄCZENIA ────────────────────────────────────────────────────────
L = @(a,b) add_line(mdl,a,b,'autorouting','smart');

L('h_ref/1','e/1');
L('e/1','K_LQR/1');
L('K_LQR/1','offset/1');
L('Q0/1','offset/2');
L('offset/1','SatPompy/1');
L('SatPompy/1','bilans/1');

% Q_in do Scope_Q (podpięcie z wyjścia saturacji, rozgałęzienie)
add_line(mdl,'SatPompy/1','Scope_Q/1','autorouting','smart');

L('bilans/1','invA/1');
L('invA/1','Int_h/1');

% h → sprzężenie zwrotne do sumatora błędu + do Q_out + do scope
add_line(mdl,'Int_h/1','e/2','autorouting','smart');
add_line(mdl,'Int_h/1','g2/1','autorouting','smart');
add_line(mdl,'Int_h/1','Scope_h/1','autorouting','smart');
add_line(mdl,'h_ref_vis/1','Scope_h/2','autorouting','smart');

% Q_out
L('g2/1','sqrtH/1');
L('sqrtH/1','muAv/1');
L('muAv/1','bilans/2');

% zakłócenie
L('d/1','bilans/3');

% To Workspace – logowanie sygnałów
add_line(mdl,'Int_h/1','Log_h/1','autorouting','smart');
add_line(mdl,'SatPompy/1','Log_Qin/1','autorouting','smart');
add_line(mdl,'muAv/1','Log_Qout/1','autorouting','smart');

%% ── 8. PARAMETRY WORKSPACE (potrzebne do symulacji) ──────────────────────
assignin('base','g',g); assignin('base','A',A); assignin('base','H',H);
assignin('base','A_v',A_v); assignin('base','mu_v',mu_v);
assignin('base','h_ref',h_ref); assignin('base','Q_in_0',Q_in_0);
assignin('base','K_lqr',K_lqr); assignin('base','Q_pump',Q_pump);
assignin('base','h0',h0);

save_system(mdl);
fprintf('Model zapisany: %s.slx\n',mdl);
fprintf('Uruchom: sim(''%s'')\n',mdl);
