sys = tf([-0.02468 -0.01602], [1 5.092 2.754]);
pidTuner(sys, 'PID')

% % Use auto pidtuner
% % [C, info] = pidtune(sys, 'PID');
% % disp(C)
% 
% % Manual
% Kp = 1.0;
% Ki = 0.0;
% Kd = 0.0;
% C = pid(Kp, Ki, Kd);
% 
% tf(C)
% 
% T = feedback(C*sys, 1);  % Closed-loop transfer function
% step(T)
% title('Closed-loop Step Response with PID')