%% MAE 506: Final Project - Solar Pump Control System
%  Features: Separate Plots per Scenario + GUI Metrics Table
%  Scenarios: Sunny (Steady), Cloudy (Spike), Overcast (Variable)

clear; clc; close all;

%% 1. SYSTEM PARAMETERS & MODELING
R = 1.2; L = 0.05; Kt = 0.15; Ke = 0.15;
J = 0.023; B = 0.012; K_flow = 3.5e-6;
K_load = 0.0005; 
Q_target_Lmin = 15;
Q_ref = Q_target_Lmin / 60000;

% State-Space Matrices
A = [-R/L, -Ke/L; Kt/J, -B/J];
B_mat = [1/L; 0]; C = [0, K_flow]; D = 0;
E_mat = [0; -1/J];

%% 2. MANDATORY ANALYSIS (Still printed to terminal for copying)
fprintf('======================================================\n');
fprintf('     SECTION 1: MATHEMATICAL MODELING (Copy to Report)\n');
fprintf('======================================================\n');
fprintf('1. DIFFERENTIAL EQUATIONS:\n');
fprintf('   Electrical: %.2f*(di/dt) + %.2f*i + %.2f*w = V\n', L, R, Ke);
fprintf('   Mechanical: %.2f*(dw/dt) + %.2f*w = %.2f*i - Load\n', J, B, Kt);
fprintf('2. STATE SPACE:\n');
disp('A ='); disp(A); disp('B ='); disp(B_mat);
fprintf('3. STABILITY:\n');
poles = eig(A);
if all(real(poles) < 0), fprintf('   Result: STABLE (Poles: %.2f, %.2f)\n', poles(1), poles(2)); end
fprintf('======================================================\n\n');

%% 3. CONTROLLER DESIGN
% LQR
Q_lqr = diag([1, 0.001]); R_lqr = 1;
[K_lqr, ~, ~] = lqr(A, B_mat, Q_lqr, R_lqr);
Ki_lqr = 50000;

% PI & Predictive
Kp = 20000; Ki = 45000; K_ff = 0.70;

%% 4. SIMULATION LOOP (SEPARATE FIGURES)
dt = 0.001; T_sim = 10; time = 0:dt:T_sim; N = length(time);

% Scenario Definitions
Load_1 = 200 * ones(1, N); % Sunny
Load_2 = 200 * ones(1, N); Load_2(time > 3 & time < 6) = 800; % Cloudy
rng(42); noise = 50 * randn(1, N); variation = 100 * sin(0.5 * time);
Load_3 = max(0, 400 + variation + noise); % Overcast

Scenarios = [Load_1; Load_2; Load_3];
Titles = {'Scenario 1: Sunny (Steady)', 'Scenario 2: Cloudy (Disturbance)', 'Scenario 3: Overcast (Variable)'};

% Data storage for the Table
Table_Data = zeros(9, 3); 

for s = 1:3
    Current_Load = Scenarios(s, :);
    
    % Initialize
    x_lqr = [0;0]; int_lqr = 0; y_lqr_rec = zeros(1,N);
    x_std = [0;0]; int_std = 0; y_std_rec = zeros(1,N);
    x_pred = [0;0]; int_pred = 0; y_pred_rec = zeros(1,N);
    
    for k = 1:N-1
        tau_d = K_load * Current_Load(k);
        
        % LQR
        u_lqr = max(0, min(24, -K_lqr*x_lqr + Ki_lqr*int_lqr));
        dx_lqr = A*x_lqr + B_mat*u_lqr + E_mat*tau_d;
        x_lqr = x_lqr + dx_lqr*dt;
        y_lqr_rec(k+1) = C*x_lqr;
        int_lqr = int_lqr + (Q_ref - y_lqr_rec(k+1))*dt;
        
        % Std PI
        e_std = Q_ref - C*x_std;
        u_std = max(0, min(24, Kp*e_std + Ki*int_std));
        dx_std = A*x_std + B_mat*u_std + E_mat*tau_d;
        x_std = x_std + dx_std*dt;
        y_std_rec(k+1) = C*x_std;
        int_std = int_std + e_std*dt;
        
        % Pred PI
        e_pred = Q_ref - C*x_pred;
        w_req = Q_ref / K_flow;
        V_ff = R*((B*w_req + tau_d)/Kt) + Ke*w_req;
        u_pred = max(0, min(24, (Kp*e_pred + Ki*int_pred) + K_ff*V_ff));
        dx_pred = A*x_pred + B_mat*u_pred + E_mat*tau_d;
        x_pred = x_pred + dx_pred*dt;
        y_pred_rec(k+1) = C*x_pred;
        int_pred = int_pred + e_pred*dt;
    end
    
    % Convert to L/min
    Y1 = y_lqr_rec * 60000;
    Y2 = y_std_rec * 60000;
    Y3 = y_pred_rec * 60000;
    
    % --- PLOT IN SEPARATE WINDOWS ---
    figure(s); set(gcf, 'Color', 'w', 'Position', [50+(s*30) 500-(s*30) 800 500]);
    
    subplot(2,1,1);
    plot(time, Y1, 'r--', 'LineWidth', 2); hold on;
    plot(time, Y2, 'b', 'LineWidth', 1.5);
    plot(time, Y3, 'g', 'LineWidth', 1.5);
    yline(Q_target_Lmin, 'k:', 'Target');
    title([Titles{s} ' - Flow Response']);
    ylabel('Flow [L/min]'); legend('LQR', 'Std PI', 'Pred PI', 'Location','Southeast');
    grid on; ylim([0 20]);
    
    subplot(2,1,2);
    area(time, Current_Load, 'FaceColor', [0.8 0.8 0.8]);
    ylabel('Load Index'); title('Disturbance Profile');
    grid on; ylim([0 1000]);
    
    % --- CALCULATE METRICS FOR THIS SCENARIO ---
    i_lqr = stepinfo(Y1, time, Q_target_Lmin);
    i_std = stepinfo(Y2, time, Q_target_Lmin);
    i_pred = stepinfo(Y3, time, Q_target_Lmin);
    
    % Store in Matrix (Rows = Rise, Over, Settle)
    base_row = (s-1)*3;
    Table_Data(base_row+1, :) = [i_lqr.RiseTime, i_std.RiseTime, i_pred.RiseTime];
    Table_Data(base_row+2, :) = [i_lqr.Overshoot, i_std.Overshoot, i_pred.Overshoot];
    Table_Data(base_row+3, :) = [i_lqr.SettlingTime, i_std.SettlingTime, i_pred.SettlingTime];
end

%% 5. GENERATE GUI METRICS TABLE
f = figure('Name', 'Performance Metrics Table', 'NumberTitle', 'off', ...
           'Position', [600 300 750 350], 'Color', 'w');

% Define Row Names
Row_Names = { ...
    'SCENARIO 1 (Sunny) - Rise Time [s]', ...
    'SCENARIO 1 (Sunny) - Overshoot [%]', ...
    'SCENARIO 1 (Sunny) - Settling Time [s]', ...
    'SCENARIO 2 (Cloudy) - Rise Time [s]', ...
    'SCENARIO 2 (Cloudy) - Overshoot [%]', ...
    'SCENARIO 2 (Cloudy) - Settling Time [s]', ...
    'SCENARIO 3 (Overcast) - Rise Time [s]', ...
    'SCENARIO 3 (Overcast) - Overshoot [%]', ...
    'SCENARIO 3 (Overcast) - Settling Time [s]' ...
};

% Create the Table
t = uitable(f, 'Data', Table_Data, ...
            'ColumnName', {'LQR (State Space)', 'Standard PI', 'Predictive PI'}, ...
            'RowName', Row_Names, ...
            'Position', [20 20 710 310], ...
            'FontSize', 10);
        
% Auto-adjust column width logic (approximate)
t.ColumnWidth = {150, 150, 150};

fprintf('Plots generated in Figures 1-3.\n');
fprintf('Metrics Table generated in Figure 4.\n');