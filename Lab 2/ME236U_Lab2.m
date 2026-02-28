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
    plot(data{i}.Time, data{i}.AccelZ);
    hold on
    grid on
    xlabel("Time (s)")
    ylabel("Accel Z (m/s^2)")
    
    xline(data{i}.Time(idxStart(i)), 'r--')
    xline(data{i}.Time(idxEnd(i)), 'r--')
    title("Radians per second: " + cmdSpd(i));
    
    matTemp = data{i}.AccelZ(idxStart(i):idxEnd(i));
    avgAccelZ(i) = sum(matTemp)/length(matTemp);
    yline(avgAccelZ(i), 'k', 'LineWidth', 1.5)
    
    sinBeta(i) = avgAccelZ(i)/g;
    Fp(i) = (m_r * g *sinBeta(i) * (l_r/l) + m_b*g*sinBeta(i))/4;
end

%%% Fit data from cmdSpd and thrust to 2nd order polynomial
p = polyfit(cmdSpd(:), Fp(:), 2); 

cmdSpd_fit = linspace(min(cmdSpd), max(cmdSpd), 100);
Fp_fit = polyval(p, cmdSpd_fit);

figure;
hold on
grid on
plot(cmdSpd, Fp, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Experimental Data');
plot(cmdSpd_fit, Fp_fit, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('Linear Fit: y = %.2e x + %.2e', p(1), p(2)));

xlabel("Motor Cmd (rad/s)")
ylabel("Thrust per propeller (N)")
title("Thrust vs Motor Command")
legend('Location', 'best')
fprintf('Linear Fit Equation: Thrust = %g * (Motor Cmd) + %g\n', p(1), p(2));


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