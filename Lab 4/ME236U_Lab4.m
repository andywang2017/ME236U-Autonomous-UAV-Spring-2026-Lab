clear; clc; close all;

logFilesProb4 = {'4-1 Test 1.csv', '4-1 Test 2.csv', '4-1 Test 3.csv'};
logFilesProb5 = {'4-2 Test 1.csv'};
logFilesProb6 = {'4-3 Test 1.csv', '4-3 Test 2.csv', '4-3 Test 3.csv'};
logFilesProb7 = {'4-4 Test 1.csv', '4-4 Test 2.csv', '4-4 Test 3.csv'};

%% Problem 4

figure
subplot(2,1,1)
hold on
grid on
for i = 1
    data = extractData(logFilesProb4{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
    plot(data.Time, data.DebugValue9, DisplayName = ...
        'Run ' + string(i) + ' Desired Angle', ...
        LineWidth = 1.5)
end
xlabel('Time (s)')
ylabel('Pitch Angle (rads)')
title('Pitch Angle vs Time (Run 1)')
legend('Location', 'best')


subplot(2,1,2)
hold on
grid on
for i = 1
    data = extractData(logFilesProb4{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
    plot(data.Time, data.DebugValue9, DisplayName = ...
        'Run ' + string(i) + ' Desired Angle', ...
        LineWidth = 1.5)
    settledRange = data.DebugValue1(2333:2815);
    settledValue = mean(settledRange);
    yline(settledValue, LineWidth = 1.5, DisplayName = 'Settled Value')
    riseT = (data.Time(2333) - data.Time(2237))* 0.9;
end
xlim([17,20])
xlabel('Time (s)')
ylabel('Pitch Angle (rads)')
title(sprintf(['Detailed Pitch Angle vs Time (Run 1)\n' ...
               'Settled Value : %.3f rads\n' ...
               'Rise Time: %.2f s'], settledValue, riseT))
legend('Location', 'best')

%% Problem 5
figure
hold on
grid on
for i = 1
    data = extractData(logFilesProb5{i});
    plot(data.Time, data.RateGyroY, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
    plot(data.Time, data.DebugValue7, DisplayName = ...
        'Run ' + string(i) + ' Desired Anglular Velocity', ...
        LineWidth = 1.5)
    time24 = abs(data.Time(1087) - data.Time(1095));
    time48 = abs(data.Time(1095) - data.Time(1108));
    time816 = abs(data.Time(1108) - data.Time(1118));
    time1632 = abs(data.Time(1118) - data.Time(1130));
end
xlabel('Time (s)')
ylabel('Pitch Angular Velocity (rads/s)')
title('Pitch Angular Velocity vs Time (Run 1)')
legend('Location', 'best')

fprintf('-------------Problem 5 Analysis--------------\n');
fprintf('2 - 4 rads/s:  %.6f s\n', time24);
fprintf('4 - 8 rads/s:  %.6f s\n', time48);
fprintf('8 - 16 rads/s:  %.6f s\n', time816);
fprintf('16 - 32 rads/s:  %.6f s\n', time1632);

%% Problem 6
figure
subplot(3,1,1)
hold on
grid on
for i = 3
    data = extractData(logFilesProb6{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
    plot(data.Time, data.DebugValue9, DisplayName = ...
        'Run ' + string(i) + ' Desired Angle', ...
        LineWidth = 1.5)
end
ylabel('Pitch Angle (rads)')
title('Pitch Angle vs Time (Run 3)')
legend('Location', 'best')


subplot(3,1,2)
hold on
grid on
for i = 3
    data = extractData(logFilesProb6{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
    plot(data.Time, data.DebugValue9, DisplayName = ...
        'Run ' + string(i) + ' Desired Angle', ...
        LineWidth = 1.5)
    settledRange = data.DebugValue1(2267:2815);
    settledValue = mean(settledRange);
    yline(settledValue, LineWidth = 1.5, DisplayName = 'Settled Value')
    riseT = (data.Time(2267) - data.Time(2025))* 0.9;
end
xlim([14,20])
ylabel('Pitch Angle (rads)')
title(sprintf(['Detailed Pitch Angle vs Time (Run 3)\n' ...
               'Settled Value : %.3f rads\n' ...
               'Rise Time: %.2f s , Overshoot: None'], settledValue, riseT))
legend('Location', 'best')

subplot(3,1,3)
hold on
grid on
for i = 3
    data = extractData(logFilesProb6{i});
    plot(data.Time, data.MotorCmd1, DisplayName = 'Motor Cmd 1', ...
        LineWidth = 1.5)
    plot(data.Time, data.MotorCmd2, DisplayName = 'Motor Cmd 2', ...
        LineWidth = 1.5)
    plot(data.Time, data.MotorCmd3, DisplayName = 'Motor Cmd 3', ...
        LineWidth = 1.5)
    plot(data.Time, data.MotorCmd4, DisplayName = 'Motor Cmd 4', ...
        LineWidth = 1.5)
end
xlim([14,20])
xlabel('Time (s)')
ylabel('Motor Command (-)')
title(sprintf(['Detailed Pitch Angle vs Time (Run 3)\n' ...
               ], settledValue, riseT))
legend('Location', 'best')

%% Problem 7
figure

colors = ["#0072BD", "#D95319", "#EDB120"];
constants = [0.2, 0.1, 0.05];
for i = 1:length(logFilesProb7)
    subplot(3,1,i)
    hold on
    grid on
    data = extractData(logFilesProb7{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5, Color = colors(i), LineStyle = '-')
    plot(data.Time, data.DebugValue9, DisplayName = ...
        'Run ' + string(i) + ' Desired Pitch Angle', ...
        LineWidth = 1.5, Color = colors(i), LineStyle = '--')
    legend('Location', 'best')
    ylabel('Pitch Angle (rads)')
    title(sprintf(['Pitch Angle vs Time (Run %.0f , Roll Angle Constant: ' ...
        '%0.2f)'], i, constants(i)))
end
xlabel('Time (s)')
legend('Location', 'best')

%% Function
function accelData = extractData(filename)
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve';
    
    dataTable = readtable(filename, opts);
    
    varnames = dataTable.Properties.VariableNames;
    accelData = struct();
    
    for i = 1:length(varnames)
        fieldName = matlab.lang.makeValidName(varnames{i});
        accelData.(fieldName) = dataTable.(varnames{i});
    end
end