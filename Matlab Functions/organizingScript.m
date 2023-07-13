%% Filter Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Locomotion and cadence detection using a single trunk-fixed accelerometer: 
% validity for children with cerebral palsy in daily life like conditions
%  - Anisoara Paraschiv-Ionescu et al
% 
% Pre-process Data for Peak Enhancement
%     1) Calculate the magnitude of acceleration acc_N 
%     2) Resample to 40Hz
%     3) Detrend Signal     
%     4) Lowpass Filter (FIR Filter, n=120coef, Fc=3.2Hz)
%     5) Zero-phase distortion filtfilt
%     6) Smooth and Differentiate using CWT
%     7) Extra smoothing with Savitzky-Golay filter
% 
% Selena Cho 
% 6-15-2023
%
%-------------------------------------------------------------------------%

reorientFolderPath = 'D:\Daily Headturns\Head Turn Pilot Study\Reorient\';
subfolders = dir(reorientFolderPath);
subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));

%% Preprocess %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 4
    fprintf("Loading in Subject Data\n")
%     subject = load(subfolders(i).name);
    % 1) Calculate the magnitude of acceleration acc_N
    % Sensor Data: [time, V, ML, AP, Yaw, Roll, Pitch]
    fprintf("1/7: Calculating Acceleration Norm\n")
    id = string(fieldnames(subject));
    sensor = "waist";
    accN = sqrt(subject.(id).(sensor)(:,2).^2 + subject.(id).(sensor)(:,3).^2 + subject.(id).(sensor)(:,4).^2);
d
    % 2) Resample to 40Hz
    fprintf("2/7: Resampling to 40Hz\n")
    rFs = 50;
    t = datetime(subject.(id).(sensor)(:,1), 'ConvertFrom', 'datenum');
    tt = timetable(t, accN);
    resampled_tt = retime(tt, 'regular', 'mean', 'TimeStep', seconds(1/rFs));

    % Extract the resampled datetime array and data
    rTime = resampled_tt.t;
    accNR = resampled_tt.accN;

    % 3) Detrend Signal
    fprintf("3/7: Detrend Signal\n")
    accNRD = detrend(accNR);

     % 6) Smooth and Differentiate using CWT
    fprintf("6/7: CWT\n")
    scale = 10;        % Scale for wavelet transform
    cwt_result(:,1) = cwt(accNRD, scale, 'gaus2');

    % 4) Lowpass Filter (FIR Filter, n=120coef, Fc=3.2Hz)
    fprintf("4/7: Lowpass Filter\n")
    Fs = 100;
    order = 120;       % FIR filter order
    Fc = 3.2;          % Cutoff frequency in Hz
    [b, a] = fir1(order, Fc / (Fs/2));
    
    % 5) Zero-phase distortion filtfilt
    fprintf("5/7: Filter Data\n")
    filtered_data = filtfilt(b, a, cwt_result);   
    
    % 7) Extra smoothing with Savitzky-Golay filter
    fprintf("7/7: Extra smoothing\n")
    windowSize = 3;    % Smoothing frame length for Savitzky-Golay filter
    accNRDcwt(:,1) = sgolayfilt(filtered_data, 0, windowSize);

    figure
    hold on
    plot(rTime,accNRD)
    plot(rTime,filtered_data)
    plot(rTime,accNRDcwt)
end

%% Detect Locomotion Periods %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars -except accNRDcwt accNRD filteredData subject rTime accNR
% Find all the peaks above THa
THa = 0.05;             % potential heel-strike threshold
[pks(:,1), locs(:,1)] = findpeaks(accNRDcwt);
locTime = datetime(rTime(locs),'format','HH:mm:ss');

LocPer = 0;             % Counter of locomotion periods
StepCountLocPer = 0;    % Counter of steps belonging to the locomotion period
LocFlag = 0;            % Flag signaling the start/end of locomotion period
THd = 3.5;              % At most 3.5s between heelstrike

% Variables to store results
trueLocPeriods = {};    % Cell array to store true locomotion periods
startStop = [];         % Array to store locations of periods
NstepsArr = [];         % Array to store the number of steps in each locomotion period
DlocPeriodArr = [];     % Array to store the duration of each locomotion period in minutes
delta_t = [];

% Loop through each peak in the processed signal
for i = 1:numel(pks)-1
    % Check if the peak amplitude is above the threshold
    if pks(i) > THa
        % Potential heel-strike event detected
        if LocFlag == 0
            % Start of a new locomotion period
            LocPer = LocPer + 1;
            StepCountLocPer = 1;
            LocFlag = 1;
        else
            % Increment step counter within the locomotion period
            StepCountLocPer = StepCountLocPer + 1;
             % Calculate the duration between successive peaks        
            delta_t = [delta_t; seconds(locTime(i+1) - locTime(i))];
            
            % Compare with the threshold duration
            if delta_t(end) < THd
                % Increment step counter within the locomotion period
                StepCountLocPer = StepCountLocPer + 1;
                THd = 1.5 + mean(delta_t); % Calculate average duration of previous steps
            end            
            
            % Check if the updated THd is below the initial threshold THd_init
            if THd < 3.5
                THd = 3.5; % Reset to initial threshold if below
            end
        end     
        
    else
        % Potential heel-strike event below the threshold
        if LocFlag == 1
            % End of the current locomotion period
            % Check if the locomotion period contains at least four consecutive steps
            if StepCountLocPer >= 10
                % Add the locomotion period to the results
                trueLocPeriods{end+1,1} = [locTime(i-StepCountLocPer+1:i)];
                startStop = [startStop;{locs(i-StepCountLocPer+1), locs(i)}];
                NstepsArr(end+1,1) = StepCountLocPer;
                DlocPeriodArr(end+1,1) = seconds(locTime(i) - locTime(i-StepCountLocPer+1)) / 60;  % Convert duration to minutes
            end
            
            % Reset the counters and flags for the next locomotion period
            StepCountLocPer = 0;
            LocFlag = 0;
            delta_t = [];
        end
    end
end

% Print the results
fprintf('Number of true locomotion periods: %d\n', LocPer);
fprintf('Number of steps in each locomotion period: %s\n', num2str(NstepsArr));
fprintf('Duration of each locomotion period (minutes): %s\n', num2str(DlocPeriodArr));

startStop = cell2mat(startStop);
figure
hold on
plot(accNRD)
plot(accNRDcwt)
plot(startStop(:,1),accNRDcwt(startStop(:,1)),'go')
plot(startStop(:,2),accNRDcwt(startStop(:,2)),'ro')


%% Remove Transient Acceleration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Compute the normalized cutoff frequency
fs = 100;
fn = fs/2; % Nyquist frequency
wna = 10/fn; % normalized cutoff frequency
wng = 3/fn; % normalized cutoff frequency
order = 5;

% Design the filter using the Butterworth function
[ba, aa] = butter(order, wna, 'low');
[bg, ag] = butter(order, wng, 'low');

fprintf('Filtering Signal\n')
fn1 = string(fieldnames(subject));
sensor = string(fieldnames(subject.(fn1)));
for i = 1:length(sensor)
    fprintf('\tSensor: %s\n',sensor{i});
    filterData.(fn1).(sensor{i}).time = subject.(fn1).(sensor{i})(:,1);
    % lowpass filter for accel to reduce transient acc
    filterData.(fn1).(sensor{i}).aData = filter(ba, aa, subject.(fn1).(sensor{i})(:,2:4));
    % highpass filter for gyro to remove errors due to drift 
    filterData.(fn1).(sensor{i}).gData = filter(bg, ag, subject.(fn1).(sensor{i})(:,5:7));
end

%%
figure
hold on
% plot(subject.S16.waist(:,2))
plot(filterData.S04.waist.aData(:,1))


%% Identify and Sort into Days %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Sorting into Days\n');
fn = string(fieldnames(filterData));
sensor = string(fieldnames(subject.(fn)));
for j = 1%:length(sensor)
    % find the number of days recorded
    fprintf('Sensor: %s\n',sensor{j});
    con = string(datetime(filterData.(fn).(sensor{j}).time, 'convertfrom', 'datenum', 'Format', 'd'));
    [c,ia,~] = unique(con);
    ia2 = [ia;length(filterData.(fn).(sensor{j}).time)];
    % with number of days, create empty structure to fill
    for k = 1:length(c)
        day = sprintf('%s%s', 'day', c(k));
        filterData.(fn).(sensor{j}).days.(day) = struct();
    end
    % for each day, save the corresponding data
    fn3 = fieldnames(filterData.(fn).(sensor{j}).days);
    for k = 1:length(ia2)-1
        fprintf('\t\tSeparating Day: %s\n',c(k));
        filterData.(fn).(sensor{j}).days.(fn3{k}).time = filterData.(fn).(sensor{j}).time(ia2(k):ia2(k+1),1);
        filterData.(fn).(sensor{j}).days.(fn3{k}).acc = filterData.(fn).(sensor{j}).aData(ia2(k):ia2(k+1),:);
        filterData.(fn).(sensor{j}).days.(fn3{k}).gyro = filterData.(fn).(sensor{j}).gData(ia2(k):ia2(k+1),:);
    end
end

