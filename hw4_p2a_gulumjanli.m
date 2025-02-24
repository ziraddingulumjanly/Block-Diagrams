
clc; clear; close all;

% Define system matrices
A = [1 -2; 
     0.5 -1];

B = [2; 
     2];

C = [-1 1];

% Compute the eigenvalues of A
eigenvalues = eig(A);

% Compute the controllability matrix
Co = [B, A*B];  % [B AB]

% Compute the rank of the controllability matrix
rank_Co = rank(Co);

% Display results
% disp('Matrix B:');
% disp(B);
% disp('Matrix AB:');
% disp(A*B);
disp('Controllability Matrix C_o:');
disp(Co);
disp(['Rank of Controllability Matrix: ', num2str(rank_Co)]);
disp('Eigenvalues of A:');
disp(eigenvalues);

% Check if the system is fully controllable
if rank_Co < size(A,1)
    disp('System is NOT fully controllable.');
else
    disp('System is fully controllable.');
end

