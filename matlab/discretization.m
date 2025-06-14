% =========================================================================
%       COMPLETE ANALYSIS SCRIPT FOR EL3015 CONTROL SYSTEMS PROJECT
% =========================================================================
% This script performs the full analysis workflow:
% 1. Defines the continuous system models (Plant and PID Controller).
% 2. Analyzes the uncompensated and compensated continuous systems.
% 3. Discretizes the models using the ZOH method.
% 4. Analyzes the final compensated discrete system.
% 5. Generates all plots and performance data for the report.
%
% Just press "Run" to get all the values and plots for your paper.
% =========================================================================

%% 0. Setup
clc;
clear;
close all;

%% 1. Define Continuous-Time System Models
disp('--- SECTION 1: SYSTEM DEFINITION ---');

% Define the continuous-time plant transfer function G(s)
% Source: "Tubes_Sisken_13222066_13222067.docx", Uncompensated System Analysis 
num_G = [-10.07];
den_G = [1 1806 4843];
G_s = tf(num_G, den_G);

% Define the continuous-time PID controller C(s) with derivative filter
% Source: "Tubes_Sisken_13222066_13222067.docx", Control System Design 
Kp = -450;
Ki = -200;
Kd = -10;
Tf = 0.01; % Filter time constant to make controller proper
C_s = pid(Kp, Ki, Kd, Tf);

disp('Continuous-Time Plant Model G(s):');
G_s
disp('Continuous-Time PID Controller C(s) with Filter:');
C_s

%% 2. Uncompensated System Analysis
disp('--- SECTION 2: UNCOMPENSATED SYSTEM ANALYSIS ---');

% Form the uncompensated closed-loop system (unity feedback)
T_uncomp_s = feedback(G_s, 1);

% Get performance metrics for the uncompensated system
uncomp_info = stepinfo(T_uncomp_s);
final_value_uncomp = uncomp_info.SettlingMin; % Using SettlingMin for negative gain
ess_uncomp = 1 - final_value_uncomp;

% Display performance metrics
disp('Uncompensated System Performance:');
disp(uncomp_info);
fprintf('Uncompensated Steady-State Error (ess): %.4f\n', ess_uncomp);

% Plot the step response
figure;
step(T_uncomp_s);
title('Step Response of Uncompensated System');
grid on;


%% 3. Compensated Continuous System Analysis
disp('--- SECTION 3: COMPENSATED CONTINUOUS SYSTEM ANALYSIS ---');

% Form the compensated continuous closed-loop system
T_comp_s = feedback(C_s * G_s, 1);

% Get performance metrics
comp_s_info = stepinfo(T_comp_s);
ess_comp_s = 1 - comp_s_info.Peak; % Assuming final value is peak for well-damped response

% Display performance metrics
disp('Compensated Continuous System Performance:');
disp(comp_s_info);
fprintf('Compensated Continuous Steady-State Error (ess): %.4f\n', ess_comp_s);

%% 4. Discretization
disp('--- SECTION 4: DISCRETIZATION ---');

% Define discretization parameters
fs = 50; % Sampling frequency in Hz
Ts = 1/fs; % Sampling time in seconds

disp(['Sampling Time (Ts) set to: ', num2str(Ts), ' s (', num2str(fs), ' Hz)']);
disp('Discretization method: Zero-Order Hold (ZOH)');

% Perform discretization
G_z = c2d(G_s, Ts, 'zoh');
C_z = c2d(C_s, Ts, 'zoh');

% Display the discrete models for the paper
disp(' ');
disp('>>> Resulting Discrete-Time Plant Model G(z) for Paper:');
G_z
disp('>>> Resulting Discrete-Time PID Controller C(z) for Paper:');
C_z

%% 5. Compensated Discrete System Analysis
disp('--- SECTION 5: COMPENSATED DISCRETE SYSTEM ANALYSIS ---');

% Form the compensated discrete closed-loop system
T_comp_z = feedback(C_z * G_z, 1);

% Get performance metrics
comp_z_info = stepinfo(T_comp_z);
ess_comp_z = 1 - comp_z_info.Peak;

% Display performance metrics
disp('Compensated Discrete System Performance:');
disp(comp_z_info);
fprintf('Compensated Discrete Steady-State Error (ess): %.4f\n', ess_comp_z);


%% 6. Final Comparative Analysis and Plots
disp('--- SECTION 6: FINAL COMPARATIVE PLOTS AND DATA ---');

% Generate the final comparison plot for the "Results and Analysis" section
figure;
step(T_uncomp_s, 'k-.'); % Uncompensated in black dash-dot
hold on;
step(T_comp_s, 'b-');   % Compensated Continuous in blue solid
step(T_comp_z, 'r--');  % Compensated Discrete in red dashed
hold off;

title('System Step Response Comparison');
legend('Uncompensated', 'Compensated (Continuous)', 'Compensated (Discrete, Ts=0.02s)', 'Location', 'SouthEast');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;

% Display a summary table of performance data for the paper
disp(' ');
disp('>>> DATA FOR COMPARISON TABLE IN YOUR PAPER <<<');
fprintf('\n| %-25s | %-15s | %-15s | %-15s |\n', 'Performance Metric', 'Uncompensated', 'Cont. Compensated', 'Disc. Compensated');
disp('------------------------------------------------------------------------------------------------');
fprintf('| %-25s | %-15.4f | %-15.4f | %-15.4f |\n', 'Rise Time (s)', uncomp_info.RiseTime, comp_s_info.RiseTime, comp_z_info.RiseTime);
fprintf('| %-25s | %-15.4f | %-15.4f | %-15.4f |\n', 'Settling Time (s)', uncomp_info.SettlingTime, comp_s_info.SettlingTime, comp_z_info.SettlingTime);
fprintf('| %-25s | %-15.2f | %-15.2f | %-15.2f |\n', 'Overshoot (%)', uncomp_info.Overshoot, comp_s_info.Overshoot, comp_z_info.Overshoot);
fprintf('| %-25s | %-15.4f | %-15.4f | %-15.4f |\n', 'Steady-State Error', ess_uncomp, ess_comp_s, ess_comp_z);
disp('------------------------------------------------------------------------------------------------');