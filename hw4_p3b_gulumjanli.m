clc; clear; close all;

% Given system parameters 
Ac = 150 * 1e-4;   
V = 3000 * 1e-6;   
M = 500;           
beta = 7000 * 1e5; 
Cl = 1 * 1e-8;     
Cf = 0.1 * 10;     
k = 20 * 1e-6;     

% Third-order state-space model
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0]; 
D = 0;       

% Display the original state-space matrices
disp('Original System Matrices:');
disp('A ='); disp(A);
disp('B ='); disp(B);

% Check controllability
Co = ctrb(A,B);
if rank(Co) == size(A,1)
    disp('The system is controllable.');
else
    disp('Warning: The system is NOT controllable!');
end

% Desired closed-loop eigenvalues
desired_poles = [-20, -12 + 12j, -12 - 12j];

% Compute full-state feedback gain K using pole placement
K = place(A, B, desired_poles);

% Display the feedback gain matrix
disp('State Feedback Gain Matrix (K):');
disp(K);

% Closed-loop system matrix (A_cl = A - BK)
A_cl = A - B*K;

% Compute eigenvalues of closed-loop system
eigenvalues_cl = eig(A_cl);
disp('Closed-loop Eigenvalues:');
disp(eigenvalues_cl);

% Simulating system response
TSPAN = [0 2]; 
X0 = [1; 1; 1];

% Define state derivative function
state_derivative = @(t,x) (A_cl*x);

% Solve using ODE45
[t, x] = ode45(state_derivative, TSPAN, X0);

%  Plot 
figure;
plot(t, x, 'LineWidth', 1.5);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('State Variables', 'Interpreter', 'latex', 'FontSize', 14);
title('System Response with Full-State Feedback', 'Interpreter', 'latex', 'FontSize', 16);
legend('$x_1$ (Position)', '$x_2$ (Velocity)', '$x_3$ (Pressure Difference)', ...
    'Interpreter', 'latex', 'FontSize', 12);
grid off;
set(gca, 'LineWidth', 1.2, 'XColor', 'k', 'YColor', 'k', 'FontSize', 12);



% Display Maximum Values of Each State
disp('Max Values of Each State:');
disp(['x_1 (Position): ', num2str(max(abs(x(:,1))))]);
disp(['x_2 (Velocity): ', num2str(max(abs(x(:,2))))]);
disp(['x_3 (Pressure Difference): ', num2str(max(abs(x(:,3))))]);

% % Save Figures as PDF & PNG
% savepath = pwd; % Set save directory to current folder

% %  Save Original Plot
% savename_original_pdf = fullfile(savepath, 'state_response_full_feedback.pdf');
% savename_original_png = fullfile(savepath, 'state_response_full_feedback.png');
% exportgraphics(gcf, savename_original_pdf);
% exportgraphics(gcf, savename_original_png);
% 
% % Save Improved Visualization
% savename_improved_pdf = fullfile(savepath, 'state_response_dual_axis.pdf');
% savename_improved_png = fullfile(savepath, 'state_response_dual_axis.png');
% exportgraphics(gcf, savename_improved_pdf);
% exportgraphics(gcf, savename_improved_png);