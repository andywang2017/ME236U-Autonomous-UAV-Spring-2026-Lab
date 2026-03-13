clc; clear; close all;

logFiles = {'Log-1000.csv', 'Log-1200.csv', 'Log-1400.csv', 'Log-1600.csv'};
cmdSpd = [1000, 1200, 1400, 1600];
idxStart = [2422, 3649, 5300, 1200]; % Manually determined indexes
idxEnd = [4200, 6000, 8300, 1800];

avgAccelZ = zeros(4,1);
sinBeta = zeros(4,1);
Fp = zeros(4,1);

m_r = 8.5*10^(-3);
m_b = 32*10^(-3);
l = 150*10^(-3);
l_r = 70*10^(-3);
g = 9.81;

data = cell(4,1);

figure;
for i = 1:length(cmdSpd)
    data{i} = extractData(logFiles{i});
    subplot(2,2,i)
    dataGraphed = plot(data{i}.Time, data{i}.AccelZ, DisplayName = "Data");
    hold on
    grid on
    xlabel("Time (s)")
    ylabel("Accel Z (m/s^2)")
    
    steadyStart = xline(data{i}.Time(idxStart(i)), 'r--', ...
        DisplayName = "Steady State Start");
    steadyEnd = xline(data{i}.Time(idxEnd(i)), 'r--', ...
        DisplayName = "Steady State End");
    title("Radians per second: " + cmdSpd(i));
    
    
    matTemp = data{i}.AccelZ(idxStart(i):idxEnd(i));
    avgAccelZ(i) = sum(matTemp)/length(matTemp);
    avgAccelVal = yline(avgAccelZ(i), 'k', 'LineWidth', 1.5, ...
        DisplayName = "Calculated Steady State Value");
    
    sinBeta(i) = avgAccelZ(i)/g;
    Fp(i) = (m_r * g *sinBeta(i) * (l_r/l) + m_b*g*sinBeta(i))/4;
    title(sprintf( ...
        ['Commanded: %g [rads/s]\n' ...
         'Final Steady State Acceleration: %.3g [m/s^2]'], cmdSpd(i), avgAccelZ(i)));
    if i == 1
        legend()
    end
end

%%% Fit data from cmdSpd and thrust to 2nd order polynomial
p = polyfit(cmdSpd(:), Fp(:), 2); 

cmdSpd_fit = linspace(min(cmdSpd), max(cmdSpd), 100);
Fp_fit = polyval(p, cmdSpd_fit);

figure;
hold on
grid on
plot(cmdSpd, Fp, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Experimental Data');
plot(cmdSpd_fit, Fp_fit, 'r-', 'LineWidth', 2, 'DisplayName', ...
    sprintf('2nd Order Fit: y = %.2e x^2 + %.2e x + %.2e', p(1), p(2), p(3)));

xlabel("Motor Cmd (rad/s)")
ylabel("Thrust per propeller (N)")
title("Thrust vs Motor Command")
legend('Location', 'best')


function accelData = extractData(filename)
    opts = detectImportOptions(filename);
    opts.PreserveVariableNames = true;
    
    %%% Read data into table
    dataTable = readtable(filename, opts);
    accelData = struct();
    accelData.Time   = dataTable.('Time');
    accelData.AccelX = dataTable.('Accelerometer x');
    accelData.AccelY = dataTable.('Accelerometer y');
    accelData.AccelZ = dataTable.('Accelerometer z');
end