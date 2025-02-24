clc; clear; close all;

% System Parameters 
Ac = 150 * 1e-4;   % cm^2 to m^2
V = 3000 * 1e-6;   % cm^3 to m^3
M = 500;           % kg
beta = 7000 * 1e5; % bar to N/m^2
Cl = 1 * 1e-8;     % cm^3 / (bar·sec) to m^3/(Pa·s)
Cf = 0.1 * 10;     % N·s/cm to Ns/m
k = 20 * 1e-6;     % cm^3 / (sec·volt) to m^3 / (s·V)

% Third-Order State-Space Model
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0]; 
D = 0;      

% External Load Force (f = 500N)
B_f = [0; 1/M; 0]; 
f_load = 500; 

% Compute Full-State Feedback Gain K
desired_poles = [-20, -12 + 12j, -12 - 12j]; 
K = place(A, B, desired_poles); % Gain matrix

% Closed-Loop System Dynamics
A_cl = A - B * K; 

% Simulation Setup
TSPAN = [0 2]; 
X0 = [0; 0; 0]; 

% Define Closed-Loop System Dynamics with Disturbance
state_derivative = @(t, x) (A_cl * x + B_f * f_load);

% Solve the System using ODE45
[t, x] = ode45(state_derivative, TSPAN, X0);

% Compute Steady-State Output y = C*x
y = C * x';

% Compute Steady-State Error e = r - y (Reference r = 0)
e_ss = -y(end); % Final value of y (steady-state error)
disp('Steady-State Error e (for f = 500N):');
disp(e_ss);

% Plot the Corrected Steady-State Error (System Output)
hf_output = figure;
plot(t, y, 'r-', 'LineWidth', 2); 
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('System Output $y(t)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Steady-State Error : Force $f = 500N$', 'Interpreter', 'latex', 'FontSize', 16);
grid off;
set(gca, 'LineWidth', 1.2, 'XColor', 'k', 'YColor', 'k', 'FontSize', 12);

% -----------------------------------------------------------------------
% %% Plot the Steady-State Error e
% hf_error = figure;
% plot(t, y, 'k-', 'LineWidth', 2);
% xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
% ylabel('Steady-State Error $e$', 'Interpreter', 'latex', 'FontSize', 14);
% title('Steady-State Error : Force $f = 500N$', 'Interpreter', 'latex', 'FontSize', 16);
% grid off;
% set(gca, 'LineWidth', 1.2, 'XColor', 'k', 'YColor', 'k', 'FontSize', 12);

% %% **Save Figures as PDF & PNG**
% savepath = pwd; % Set save directory to current folder
% 
% % Save System Output Plot
% savename_output_pdf = fullfile(savepath, 'steady_state_output.pdf');
% savename_output_png = fullfile(savepath, 'steady_state_output.png');
% exportgraphics(hf_output, savename_output_pdf);
% exportgraphics(hf_output, savename_output_png);

% % Save Steady-State Error Plot
% savename_error_pdf = fullfile(savepath, 'steady_state_error.pdf');
% savename_error_png = fullfile(savepath, 'steady_state_error.png');
% exportgraphics(hf_error, savename_error_pdf);
% exportgraphics(hf_error, savename_error_png);

