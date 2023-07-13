%% Turning Characteristics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     1) Filter data
%     2) Split into Days
%     3) Rotate IMUs
%     4) Identify Turning Characteristics
% 
%     -Selena Cho 7/1/2023
%
%% Load in Data and RotateIMU %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

reorientFolderPath = 'D:\Daily Headturns\Head Turn Pilot Study\Reorient\';
subfolders = dir(reorientFolderPath);
subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));

for i = 16%1:19

    % 1) Load in filteredData data
    fprintf('filteredData Number: %d\n',i);
    filteredData = load(subfolders(i).name);
end

%% Filter Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fs = 100; % sampling frequency
fcg = 3; % gyro cutoff frequency
fca = 5; % acc cutoff frequency
order = 4; % filter order

% Compute the normalized cutoff frequency
fn = fs/2; % Nyquist frequency
wng = fcg/fn; % normalized cutoff frequency
wna = fca/fn; % normalized cutoff frequency

% Design the filter using the Butterworth function
[b, a] = butter(order, wng, 'low');
[ba, aa] = butter(order, wna, 'low');

fn = string(fieldnames(filteredData));
fn2 = fieldnames(filteredData.(fn));
for i = 1:length(fn2)
    filteredData.(fn).(fn2{i}).time = filteredData.(fn).(fn2{i})(:,1);
    filteredData.(fn).(fn2{i}).acc = filtfilt(ba, aa,filteredData.(fn).(fn2{i})(:,2:4));
    filteredData.(fn).(fn2{i}).gyr = filtfilt(b, a,filteredData.(fn).(fn2{i})(:,5:7));
end

%% Split into Days %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic 
fn = string(fieldnames(filteredData));
sensor = string(fieldnames(filteredData.(fn)));
separate = struct();

for i = 1:length(sensor)
    fprintf('Sensor: %s\n', sensor(i));
    time = filteredData.(fn).(sensor{i}).time;
    con = string(datetime(time, 'convertfrom', 'datenum', 'Format', 'd'));
    [c, ia, ~] = unique(con);
    ia2 = sort([ia; length(time)]);
    num = string(1:length(c));
    
    % Preallocate structure fields
    for k = 1:length(num)
        day = sprintf('%s%s', 'day', num(k));
        separate.(fn).(sensor{i}).(day).time = [];
        separate.(fn).(sensor{i}).(day).acc = [];
        separate.(fn).(sensor{i}).(day).gyr = [];
    end
    
    for k = 1:length(num)
        day = sprintf('%s%s', 'day', num(k));
        fprintf('\t\tSeparating Day: %s\n', num(k));
        
        indices = ia2(k):ia2(k+1);
        separate.(fn).(sensor{i}).(day).time = time(indices);
        separate.(fn).(sensor{i}).(day).acc = filteredData.(fn).(sensor{i}).acc(indices, :);
        separate.(fn).(sensor{i}).(day).gyr = filteredData.(fn).(sensor{i}).gyr(indices, :);
    end
end
toc

%% Detect Head Turn in Yaw %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find Peaks >15 deg/s for a head turn
% Save the location in other sensors

fprintf('Find Ang Vel Peaks greater than 15deg/s\n');
fn = string(fieldnames(separate));
sensor = string(fieldnames(separate.(fn)));

fprintf('Finding Peak Velocity > 15 deg/s\n');
for i = 1:length(sensor)
    fprintf('\tSensor: %s\n', sensor{i})
    days = string(fieldnames(separate.(fn).(sensor{i})));
    
    for j = 1:length(days)
        gyrData = separate.(fn).(sensor{i}).(days{j}).gyr(:, 1);
        velThreshold = 15;
        minPeakWidth = 40;
        
        pLoc = [];
        nLoc = [];
        
        % Find positive peaks above the threshold
        [pPk, pLoc] = findpeaks(gyrData, 'MinPeakHeight', velThreshold, 'MinPeakWidth', minPeakWidth);
        
        % Find negative peaks above the threshold
        [nPk, nLoc] = findpeaks(-gyrData, 'MinPeakHeight', velThreshold, 'MinPeakWidth', minPeakWidth);
        
        % Combine positive and negative peak locations and sort them
        velLoc = sort([pLoc; nLoc]);
        
        % Take the absolute values of positive and negative peaks and combine them
        velPk = cat(1, abs(pPk), abs(nPk));
        
        % Store the calculated velocity locations and peaks in the separate structure
        separate.(fn).(sensor{i}).(days{j}).velLoc = velLoc;
        separate.(fn).(sensor{i}).(days{j}).velPk = velPk;
    end
end


fprintf('Finding Start&Stop Points of a full Turn\n');
for i = 1:length(sensor)
    days = string(fieldnames(separate.(fn).(sensor{i})));
    for j = 1:length(days)
        data = separate.(fn).(sensor{i}).(days{j});
        velLoc = data.velLoc;
        gyr = data.gyr;

        velLocLength = length(velLoc);
        separate.(fn).(sensor{i}).(days{j}).window = zeros(velLocLength-1, 2);

        for k = 1:velLocLength-1
            % find start of turn where it crosses zero
            n = velLoc(k);
            while n > 1 && sign(gyr(n,1)) == sign(gyr(n-1,1))
                n = n - 1;
            end
            separate.(fn).(sensor{i}).(days{j}).window(k, 1) = n;

            % find stop of turn where it crosses zero
            m = velLoc(k);
            while m > 1 && sign(gyr(m,1)) == sign(gyr(m+1,1))
                m = m + 1;
            end
            separate.(fn).(sensor{i}).(days{j}).window(k, 2) = m;
        end
    end
end


%% Check if the start and stop too close %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

w = 1;
fprintf('Check if turns start/stop too close\n');
for i = 1:length(sensor)    
    fprintf('\tSensor: %s\n',sensor{i})
    days = string(fieldnames(separate.(fn).(sensor{i})));
    for j = 1:length(days)   
%         separate.(fn).(sensor{i}).(days{j}) = rmfield(separate.(fn).(sensor{i}).(days{j}), 'newWindow');
        window = separate.(fn).(sensor{i}).(days{j}).window;
        turnGap = 30;
        newWindow = zeros(size(window));

        n = 1;
        m = 1;
        r = 1;
        rr = [];

        while m < size(window, 1)-1
            if window(m+1, 1) - window(m, 2) < turnGap
                newWindow(n, 1) = window(m, 1);
                newWindow(n, 2) = window(m+1, 2);
                rr(r,1) = m+1;
                r = r+1;
                n = n+1;
                m = m+2;
            else
                newWindow(n, 1) = window(m, 1);
                newWindow(n, 2) = window(m, 2);
                n = n+1;
                m = m+1;
            end
        end 

        if ~isempty(rr)
            separate.(fn).(sensor{i}).(days{j}).velPk(rr) = [];
            separate.(fn).(sensor{i}).(days{j}).velPk(end-2:end) = [];
            separate.(fn).(sensor{i}).(days{j}).velLoc(rr) = [];
            separate.(fn).(sensor{i}).(days{j}).velLoc(end-2:end) = [];
        end

        separate.(fn).(sensor{i}).(days{j}).newWindow = newWindow(1:n-1, :);
    end
end

%% Integrate for peak angular displacement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Integrate for Peak Displacement\n');
time = (0:(1/100):(60*60*24))';
for i = 1:length(sensor)
    fprintf('\tSensor: %s\n',sensor{i}) 
    days = string(fieldnames(separate.(fn).(sensor{i})));    
    for j = 1:length(days)
        gyr = separate.(fn).(sensor{i}).(days{j}).gyr;
        window = separate.(fn).(sensor{i}).(days{j}).newWindow;
        amplitude = zeros(length(window), 1);
        for k = 1:length(window)
            amplitude(k,1) = abs(trapz(time(window(k,1):window(k,2)),gyr(window(k,1):window(k,2))));
        end 
        separate.(fn).(sensor{i}).(days{j}).amplitude = amplitude;
    end
end

%% Remove Head Turns Less than 10 deg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Remove Head turns <10deg \n');
for i = 1:length(sensor)
    fprintf('\tSensor: %s\n',sensor{i}) 
    days = string(fieldnames(separate.(fn).(sensor{i})));    
    for j = 1:length(days)
        gyr = separate.(fn).(sensor{i}).(days{j}).gyr;
        amplitude = separate.(fn).(sensor{i}).(days{j}).amplitude;
        newAmp = zeros(length(amplitude), 1);
        r = 1;
        rr = [];
        for k = 1:length(amplitude)
            if amplitude < 10
                rr(r,1) = k;
            end
        end
        if ~isempty(rr)
            separate.(fn).(sensor{i}).(days{j}).velPk(rr) = [];
            separate.(fn).(sensor{i}).(days{j}).velPk(end-2:end) = [];
            separate.(fn).(sensor{i}).(days{j}).velLoc(rr) = [];
            separate.(fn).(sensor{i}).(days{j}).velLoc(end-2:end) = [];
            separate.(fn).(sensor{i}).(days{j}).amplitude(rr) = [];
            separate.(fn).(sensor{i}).(days{j}).amplitude(end-2:end) = [];
        end
    end
end

%% Save all the turn metrics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fn = string(fieldnames(separate));
sensor = string(fieldnames(separate.(fn)));

fprintf('Saving Metrics\n');
for i = 1:length(sensor)
    fprintf('\tSensor: %s\n', sensor{i})
    days = string(fieldnames(separate.(fn).(sensor{i})));    
    for j = 1:length(days)
        turnMetrics.(fn).(sensor{i}).(days{j}).velPk = separate.(fn).(sensor{i}).(days{j}).velPk;
        turnMetrics.(fn).(sensor{i}).(days{j}).amplitude = separate.(fn).(sensor{i}).(days{j}).amplitude;
        turnMetrics.(fn).(sensor{i}).(days{j}).duration = (separate.(fn).(sensor{i}).(days{j}).newWindow(:,2) - separate.(fn).(sensor{i}).(days{j}).newWindow(:,1))/100;
    end
end

%% 

figure
hold on
plot(separate.S16.waist.day6.gyr(:,1))
plot(separate.S16.waist.day6.velLoc,separate.S16.waist.day6.gyr(separate.S16.waist.day6.velLoc,1),'*')
plot(separate.S16.waist.day6.newWindow(:,1),separate.S16.waist.day6.gyr(separate.S16.waist.day6.newWindow(:,1),1),'g*')
plot(separate.S16.waist.day6.newWindow(:,2),separate.S16.waist.day6.gyr(separate.S16.waist.day6.newWindow(:,2),1),'r*')


%% Filter to Find Wakeup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

id = string(fieldnames(separate));
sensor = string(fieldnames(separate.(id)));

for i = 3%:length(sensor)
    day = string(fieldnames(separate.(id).(sensor{i})));
    for j = 1:length(day)
        accV = separate.(fn).(sensor{i}).(day{j}).acc(:,1) - mean(separate.(fn).(sensor{i}).(day{j}).acc(:,1));

        % Smooth by integrating
        fs = 100;
        duration = length(accV) / fs; % Total number of samples divided by the sampling frequency
        time(:,1) = linspace(0, duration, length(accV));
        accVI = cumtrapz(time,accV);

        % Differentiate via Gaussian CWT
        scale = 10;        % Scale for wavelet transform
        accVIC(:,1) = cwt(accVI, scale, 'gaus2');

        % Low-pass Butterworth, 4th order, Fc=2 Hz
        fc = 0.5; % Specify the cutoff frequency for the Butterworth filter
        [b, a] = butter(4, fc/(fs/2),"low");
        accVICB = filtfilt(b, a, accVIC);

        figure
        plot(accVICB)
        clearvars accV accVI accVIC accVICB time
    end
end

for i = 1:length(sensor)
    fprintf('\tSensor: %s\n',sensor{i})
    days = string(fieldnames(separate.(fn).(sensor{i})));
    for j = 1:length(days)
        separate.(fn).(sensor{i}).(days{j}).window = zeros(length(separate.(fn).(sensor{i}).(days{j}).velLoc)-1,2);
        for k = 1:length(separate.(fn).(sensor{i}).(days{j}).velLoc)-1
            % find start of turn where it cross zero
            n = separate.(fn).(sensor{i}).(days{j}).velLoc(k);
            while n > 1 && sign(separate.(fn).(sensor{i}).(days{j}).gyr(n,1)) == sign(separate.(fn).(sensor{i}).(days{j}).gyr(n-1,1))
                n = n-1;
            end
            separate.(fn).(sensor{i}).(days{j}).window(k,1) = n;

            % find stop of turn where it cross zero
            m = separate.(fn).(sensor{i}).(days{j}).velLoc(k);
            while m > 1 && sign(separate.(fn).(sensor{i}).(days{j}).gyr(m,1)) == sign(separate.(fn).(sensor{i}).(days{j}).gyr(m+1,1))
                m = m+1;
            end
            separate.(fn).(sensor{i}).(days{j}).window(k,2) = m;
        end
    end
end
