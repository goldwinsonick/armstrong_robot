data = readtable('../data_acquisition/test1.csv');
t = data.time - data.time(1); % start at zero
u = data.torque;
y = data.joint_angle;

% Define your manual window
t_start = 2.0;
t_end = 10.0;

idx = (t >= t_start) & (t <= t_end);

t_crop = t(idx);
u_crop = u(idx);
y_crop = y(idx);

Ts = mean(diff(t_crop));
data_id = iddata(y_crop, u_crop, Ts);

sys1 = tfest(data_id, 2, 0);
sys2 = tfest(data_id, 2);

disp('OLTF No Zero:');
sys1
disp('OLTF With Zero:');
sys2

figure;
compare(data_id, sys2);
title('Model');