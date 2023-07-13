%% Filter Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% An enhanced estimate of initial contact and final contact instants of time
% using lower trunk inertial sensor data
%     - John McCamley et al
% 
% A) Filtering acceleration signal for IC and FC
%     1) Loading in Subject Data
%     2) Calculate Acceleration Norm
%     3) Smooth by integrating
%     4) Differentiate via Gaussian CWT
%     5) Low-pass Butterworth, 4th order, Fc=2 Hz
% 
% B) Sort the filtered data into respective days
% C) Detect number of steps each day
% 
% Selena Cho
% 6-29-2023
%
%-------------------------------------------------------------------------%
%
% Time = (1) Acc(2:4) = (V,ML,AP) and Gyro(5:7) = (V,ML,AP)
reorientFolderPath = 'D:\Daily Headturns\Head Turn Pilot Study\Reorient\';
subfolders = dir(reorientFolderPath);
subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));

%% Begin the filtering process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars -except dailySteps subfolders i
for i = 18%1:19
    steps = 5;

    % 1) Load in subject data
    fprintf('Subject Number: %d\n',i);
%     f = waitbar(0,'Loading in subject data');
    subject = load(subfolders(i).name);

    % 2) Calculate Acceleration Norm
%     waitbar(1/steps,f,'Calculation Acc Norm + Acc V Drift Removal');
    id = string(fieldnames(subject));
    sensor = "neck";
    % Acceleration norm
    accN = sqrt(subject.(id).(sensor)(:,2).^2 + subject.(id).(sensor)(:,3).^2 + subject.(id).(sensor)(:,4).^2);
    % Correction for drift
    accV = subject.(id).(sensor)(:,2) - mean(subject.(id).(sensor)(:,2));

    % 3) Smooth by integrating
%     waitbar(2/steps,f,'Integrating');
    fs = 100;
    duration = length(subject.(id).(sensor)) / fs; % Total number of samples divided by the sampling frequency
    time = linspace(0, duration, length(subject.(id).(sensor)));
    accNI = cumtrapz(time,accN);
    accVI = cumtrapz(time,accV);

    % 4) Differentiate via Gaussian CWT
%     waitbar(3/steps,f,'Differentiate vis Gaussian CWT');
    scale = 10;        % Scale for wavelet transform
    accVIC(:,1) = cwt(accVI, scale, 'gaus2');
    
    % 5) Low-pass Butterworth, 4th order, Fc=2 Hz
%     waitbar(4/steps,f,'Low=pass Butterworth');
    fc = 2; % Specify the cutoff frequency for the Butterworth filter
    [b, a] = butter(4, fc/(fs/2));
    accVICB = filtfilt(b, a, accVIC);

%     waitbar(5/steps,f,'Completed Filtering')
%     pause(1)
%     close(f)
    subject.(id).accVICB = accVICB;

    % Identify and Sort into Days %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % VertAcc is based from Waist Sensor
    fprintf('Sorting into Days\n');
    fn = string(fieldnames(subject));
%     sensor = "waist";
    con = string(datetime(subject.(fn).(sensor)(:,1), 'convertfrom', 'datenum', 'Format', 'd'));
    [c,ia,~] = unique(con);
    ia2 = sort([ia;length(subject.(fn).(sensor)(:,1))]);
    num = string(linspace(1,length(c),length(c)));
    % with number of days, create empty structure to fill
    for k = 1:length(num)
        day = sprintf('%s%s', 'day', num(k));
        separate.(fn).(day) = struct();
    end
    fn3 = fieldnames(separate.(fn));
    for k = 1:length(ia2)-1
        fprintf('\t\tSeparating Day: %s\n',num(k));
        separate.(fn).(fn3{k}).time = subject.(fn).(sensor)(ia2(k):ia2(k+1),1);
        separate.(fn).(fn3{k}).accVICB = subject.(fn).accVICB(ia2(k):ia2(k+1),1);
    end    
    
    %% Step Detection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    fprintf('Step Detection\n');
    fn = string(fieldnames(separate));
    fn2 = string(fieldnames(separate.(fn)));
    for d = 1:length(fn2)
        % Threshold for peak selection
        THa = 0.02; % g
        
        % Variables initialization
        LocPer = 0; % Counter of locomotion periods
        StepCountLocPer = 0; % Counter of steps belonging to the locomotion period
        LocFlag = 0; % Flag signaling the start/end of locomotion period
        THd = 3.5; % Threshold for comparison of duration(s) between successive peaks (initial value)
        
        % Peak selection
        [~,peaks] = findpeaks(-separate.(fn).(fn2{d}).accVICB, 'MinPeakHeight',THa);
        stepCount = 0;
        delta_t = [];
        stepBout = [];
        boutDur = [];

        % Loop through the selected peaks
        for p = 1:length(peaks)-1
            % Duration between successive peaks
            try
                delta_t = (peaks(d+1) - peaks(d))/100;                
                % Compare duration with threshold
                if ((peaks(p+1) - peaks(p))/100) < THd 
                    if LocFlag == 0
                        LocPer = LocPer + 1;
                        LocFlag = 1;
                        StepCountLocPer = StepCountLocPer + 1;
                    else
                        StepCountLocPer = StepCountLocPer + 1;
                        THd = 1.5 + mean(delta_t);
                        if THd < 1.5
                            THd = 3.5; % Reset to initial threshold if below
                        elseif THd > 5
                            THd = 3.5;
                        end
                    end 
                elseif StepCountLocPer < 5
    
                else    
                    % End of Locomotion period
                    LocFlag = 0;
                    % Reset step counter
                    stepBout = [stepBout;StepCountLocPer];
                    stepCount = stepCount+StepCountLocPer;
                    StepCountLocPer = 0;
                end   
            catch                 
            end             
        end
        newCount = stepCount/round(length(separate.(fn).(fn2{d}).accVICB)/60/60/100);
        bouts = LocPer/round(length(separate.(fn).(fn2{d}).accVICB)/60/60/100);
%         fprintf('Bouts per Hour: %d\n', round(bouts));
%         fprintf('Steps per Hour: %d\n', round(newCount));
%         fprintf('Steps per Bout: %d\n', round(mean(stepBout)));
        dailySteps.(fn).stepsTotal(d,1) = round(24*stepCount/round(length(separate.(fn).(fn2{d}).accVICB)/60/60/100));
        dailySteps.(fn).stepsHour(d,1) = round(newCount);
        dailySteps.(fn).boutsHour(d,1) = round(bouts);
        dailySteps.(fn).stepBout(d,1) = round(mean(stepBout));
        LocPer = 0;
        stepCount = 0;  
    end
    %%
    clearvars -except dailySteps subfolders i
end

%% Plot Number of Steps %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fn = fieldnames(dailySteps);
% figure
for i = 1:length(fn)
%     hold on
    fn2 = fieldnames(dailySteps.(fn{i}));
    for j = 4
        maxSteps(i,1) = max(dailySteps.(fn{i}).(fn2{j})(2:end));
%         plot(dailySteps.(fn{i}).(fn2{j})(2:end),'*-')
    end
end


%%
figure
hold on
plot(separate.(fn).(fn2{d}).accVICB)
plot(peaks,separate.(fn).(fn2{d}).accVICB(peaks),'*')


for d = 1:length(fn2)
    avd(d,1) = mean(separate.(fn).(fn2{d}).accVICB)+ 3*std(separate.(fn).(fn2{d}).accVICB)
end


