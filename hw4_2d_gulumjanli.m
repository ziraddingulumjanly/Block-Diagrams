

clc; clear; close all;

% Define the system matrices
A = [1 -2; 
     0.5 -1];

B = [4.1; 2]; 


C =[-1 1];

omega_0 = 2;   
zeta = 0.707;  

% Compute the desired closed-loop poles
s1 = -zeta*omega_0 + 1j*omega_0*sqrt(1 - zeta^2);
s2 = -zeta*omega_0 - 1j*omega_0*sqrt(1 - zeta^2);
desired_poles = [s1, s2];

% Compute the full-state feedback gain using pole placement
[K, prec] = place(A, B, desired_poles);

% Display the results
disp('Full-State Feedback Gain K:');
disp(K);
disp('Pole Placement Accuracy Estimate:');
disp(prec);

%Simulating System Response
TSPAN = [0 10]; % Time span for simulation
X0 = [1; 0]; % Initial condition

% Define state-space representation with feedback control
closed_loop_dynamics = @(t, x) (A - B*K) * x;

% Solve the system using ODE45
[t, x] = ode45(closed_loop_dynamics, TSPAN, X0);

% Plot the System Response
figure;
plot(t, x(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'b', 'LineWidth', 1.5);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('State Variables', 'Interpreter', 'latex', 'FontSize', 14);
title('Closed-Loop System Response', 'Interpreter', 'latex', 'FontSize', 14);
legend({'$x_1$', '$x_2$'}, 'Interpreter', 'latex', 'FontSize', 12);
grid on;

%% Save the Figure
saveas(gcf, 'closed_loop_response.png');
