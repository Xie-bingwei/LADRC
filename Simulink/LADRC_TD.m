%% Initialization
clear; close all; clc;

%% Parameter settings
Ts = 0.001; % Sampling time
EndTime = 10; % End time
t = 0:Ts:EndTime; % Time vector

% TD parameters
omega = 0.8; % Cutoff frequency
Td = 1/(sqrt(2)*omega); % Time delay
td_error = 0; % Initialize tracking error

%% Generate reference signal
r = sin(t);

%% TD simulation
e_td = zeros(1, length(t));
y_td = zeros(1, length(t));
for k = 2:length(t)
    % Calculate tracking error
    td_error_dot = (r(k) - y_td(k-1) - 2 * (1 - Td * omega) * td_error) / (Td ^ 2);
    td_error = td_error + Ts * td_error_dot;
    
    % Update tracking differentiator output
    y_td(k) = r(k) - td_error;
end

%% Visualize results
figure;
plot(t, r, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_td, 'r', 'LineWidth', 1.5);
grid on; xlabel('Time(s)'); ylabel('Signal amplitude');
ylim([-3, 3]);
title('TD'); legend('Reference signal', 'Tracking differentiator output');
