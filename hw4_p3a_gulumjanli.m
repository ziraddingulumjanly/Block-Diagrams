clc; clear; close all;

% Given system parameters
% System Parameters (Converted to SI Units)
Ac = 150 * 1e-4;   % cm^2 to m^2
V = 3000 * 1e-6;   % cm^3 to m^3
M = 500;           % kg
beta = 7000 * 1e5; % bar to N/m^2
Cl = 1 * 1e-8;     % cm^3 / (bar·sec) to m^3/(Pa·s)
Cf = 0.1 * 10;     % N·s/cm to Ns/m
k = 20 * 1e-6;     % cm^3 / (sec·volt) to m^3 / (s·V)

% Define the state-space matrices for the third-order system
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0]; 
D = 0;       

% Display the system matrices
disp('State-space representation (Reduced 3rd Order System):');
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);

% Compute the eigenvalues of A
eigenvalues = eig(A);

% Display the eigenvalues
disp('Eigenvalues of the reduced system:');
disp(eigenvalues);
