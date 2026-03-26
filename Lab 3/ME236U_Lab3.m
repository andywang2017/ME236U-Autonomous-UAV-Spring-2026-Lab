clear; clc; close all;

logFilesProb4 = {'P4-1.csv', 'P4-2.csv', 'P4-3.csv','P4-4.csv','P4-5.csv'};
logFilesProb5 = {'P5-1.csv', 'P5-2.csv', 'P5-3.csv','P5-4.csv','P5-5.csv'};

%% Problem 4
subplot(1,2,1)
hold on
grid on
for i = 1:length(logFilesProb4)
    data = extractData(logFilesProb4{i});
    plot(data.Time, data.DebugValue1, DisplayName = 'Run ' + string(i), ...
        LineWidth = 1.5)
end
yline(deg2rad(30), DisplayName = '30 Degrees');
xlabel('Time (s)')
ylabel('Pitch Estimate (rads)')
title('Pitch Estimate vs Time')
legend('Location', 'best')

subplot(1,2,2)
hold on
grid on
for i = 1:length(logFilesProb4)
    data = extractData(logFilesProb4{i});
    plot(data.Time, data.RateGyroY, DisplayName = 'Run ' + string(i))
end
xlabel('Time (s)')
ylabel('Rate Gyro Y (rad/s)')
title('Rate Gyro Y vs Time')
legend('Location', 'best')

%% Problem 5
dataP5 = extractData(logFilesProb5{2});

figure
subplot(1,3,1)
plot(dataP5.Time, dataP5.DebugValue0)
grid on
xlabel('Time (s)')
ylabel('Roll Estimate')
title('Roll Estimate vs Time')
ylim([-1.5,1.5])
startVals = mean(dataP5.DebugValue0(1:5)); 
endVals = mean(dataP5.DebugValue0(end-5:end-1));
title(sprintf(['Roll Estimate vs Time \n' ...
    '(Start: %.5f, End: %.5f)'], startVals, endVals))
yur = xline(15.73, DisplayName = 'Motion End');
legend(yur, 'Location', 'best')

subplot(1,3,2)
plot(dataP5.Time, dataP5.DebugValue1)
grid on
xlabel('Time (s)')
ylabel('Pitch Estimate')
title('Pitch Estimate vs Time')
ylim([-1.5,1.5])
startVals = mean(dataP5.DebugValue1(1:5)); 
endVals = mean(dataP5.DebugValue1(end-5:end-1));
title(sprintf(['Pitch Estimate vs Time \n' ...
    '(Start: %.5f, End: %.5f)'], startVals, endVals))
yur = xline(15.54, DisplayName = 'Motion End');
legend(yur, 'Location', 'best')


subplot(1,3,3)
plot(dataP5.Time, dataP5.DebugValue2)
grid on
xlabel('Time (s)')
ylabel('Yaw Estimate')
title('Yaw Estimate vs Time')
ylim([-1.5,1.5])
startVals = mean(dataP5.DebugValue2(1:5)); 
endVals = mean(dataP5.DebugValue2(end-5:end-1));
title(sprintf(['Yaw Estimate vs Time \n' ...
    '(Start: %.5f, End: %.5f)'], startVals, endVals))
yur = xline(15.55, DisplayName = 'Motion End');
legend(yur, 'Location', 'best')


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