clear; clc; close;
% Apply soft clipping using tanh function - cubic algorithm
% with Adjustable with two parameters: Factor and Level.

% fs=44100, Amplitude=1, DC-offset=0
%sine frequeny = 440; duration = 5 seconds;
[x, t] = singen(44100,1,0,440,5);

% default parameters for level and factor
default_level = 1;
default_factor = 3;
figure();
%%
% Adjust factor for softer-harder clipping
factors = [3 5 7 9 11]; 

x_clipped_with_factors = zeros(length(factors),length(x));
for i=1:length(factors)
   % x_clipped_with_factors(i, :) = cubicDistortion(x, default_level, i);
    x_clipped_with_factors(i, :) = cubicDistortion(x, default_level, i);
end

% Plot original and clipped signals
subplot(1,2,1)
plot(t, x, 'k', t, x_clipped_with_factors); yline(0, 'r--');


xlabel('Time'); ylabel('Amplitude');
title('Soft Clipping of Sine Wave with Different Factors');
grid on; grid minor; xlim([0 2/440]);

% Adjust size and position of legend
lgd1 = legend('Original', 'Factor = 3', 'Factor = 5', 'Factor = 7', 'Factor = 9', 'Factor = 11');
lgd1.FontSize = 14; % Set font size
lgd1.Position = [0.317, 0.727, 0.2, 0.2]; % Set position [left, bottom, width, height]

%%
% Adjust level for amount of distortion
levels = 0:0.2:1;
x_clipped_with_levels = zeros(length(levels), length(x));
for i=1:length(levels)
        x_clipped_with_levels(i, :) = cubicDistortion(x, levels(i), 1);
end

subplot(1,2,2);
plot(t, x_clipped_with_levels); yline(0, 'r--')
xlabel('Time'); ylabel('Amplitude');
title('Soft Clipping of Sine Wave with Different Levels');
grid on; grid minor; xlim([0 2/440]);

% Adjust size and position of legend
lgd2 = legend('Level = 0', 'Level = 0.2', 'Level = 0.4', 'Level = 0.6', 'Level = 0.8', 'Level = 1');
lgd2.FontSize = 14; % Set font size
lgd2.Position = [0.758, 0.727, 0.2, 0.2]; % Set position [left, bottom, width, height]
