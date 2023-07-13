%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%             The purpose of this script is to pull in the raw data:
%                 1) resampleCWA: resamples data to 100Hz, for some
%                     reason, the IMU records replicates
%                 2) Organize the axes in a readable way:
%                         Head sensor accel (V,ML,AP)=(ax,az,-ay)
%                         Neck+Lumbar accel (V,ML,AP)=(ay,-ax,az)

%% Set subject folder path %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mainPath = 'D:\Daily Headturns\Head Turn Pilot Study\';
subjectFolderPath = 'D:\Daily Headturns\Head Turn Pilot Study\Subjects';

% This resamples and removes replications of datapoints 
% Retrieve subfolders in the subject folder
subfolders = dir(subjectFolderPath);
subfolders = subfolders([subfolders.isdir]);
subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));

% Create 'resample' folder within the 'Pilot Data' directory
resampleFolderPath = fullfile(mainPath, 'Resample');
if ~isfolder(resampleFolderPath)
    mkdir(resampleFolderPath);
end

% Resample and Create Folder to save Resample Data
for subjectIndex = 1:numel(subfolders)
    subjectFolder = subfolders(subjectIndex);
    subjectName = subjectFolder.name;
    
    fprintf('Subject: %s\n', subjectName);
    
    % Retrieve IMU files in the subject folder
    imuFiles = dir(fullfile(subjectFolder.folder, subjectFolder.name, '*.cwa'));
    
    for imuIndex = 1:numel(imuFiles)
        imuFile = imuFiles(imuIndex);
        imuName = extractBetween(imuFile.name, 13, '.cwa');
        
        fprintf('\tSensor: %s\n', string(imuName));
        
        % Resample data to 100Hz
        imuFilePath = fullfile(imuFile.folder, imuFile.name);
        subjectData = resampleCWA(imuFilePath, 100);
        
        % Store data in the subject structure
        subject.(string(subjectName)).(string(imuName)) = subjectData; 
    end

end

%% Reorient the sensors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mainPath = 'D:\Daily Headturns\Head Turn Pilot Study\';
resamplePath = 'D:\Daily Headturns\Head Turn Pilot Study\Resample\';
subfolders = dir(resamplePath);
subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));

reorientFolderPath = fullfile(mainPath, 'Reorient');
if ~isfolder(reorientFolderPath)
    mkdir(reorientFolderPath);
end

% Reorient the sensors
% rearrange accel data to (V,ML,AP) to rotate sensor
% Head sensor accel (V,ML,AP)=(ax,az,-ay)
% Neck+Lumbar accel (V,ML,AP)=(ay,-ax,az)
for i = 3:length(subfolders)
    subject = load(subfolders(i).name);    
    fn = string(fieldnames(subject));
    fprintf("Subject: %s\n",fn)
    sensor = fieldnames(subject.(fn));
    for j = 1:length(sensor)
        fprintf("\tSensor: %s\n",string(sensor(j)))
        if strcmp(sensor(j),'head')
            reorient.(fn).(sensor{j})(:,1) = subject.(fn).(sensor{j})(:,1);
            reorient.(fn).(sensor{j})(:,2) = subject.(fn).(sensor{j})(:,2);
            reorient.(fn).(sensor{j})(:,3) = subject.(fn).(sensor{j})(:,4);
            reorient.(fn).(sensor{j})(:,4) = -subject.(fn).(sensor{j})(:,3);
            reorient.(fn).(sensor{j})(:,5) = subject.(fn).(sensor{j})(:,5);
            reorient.(fn).(sensor{j})(:,6) = subject.(fn).(sensor{j})(:,7);
            reorient.(fn).(sensor{j})(:,7) = -subject.(fn).(sensor{j})(:,6);
        else
            reorient.(fn).(sensor{j})(:,1) = subject.(fn).(sensor{j})(:,1);
            reorient.(fn).(sensor{j})(:,2) = subject.(fn).(sensor{j})(:,3);
            reorient.(fn).(sensor{j})(:,3) = -subject.(fn).(sensor{j})(:,2);
            reorient.(fn).(sensor{j})(:,4) = subject.(fn).(sensor{j})(:,4);
            reorient.(fn).(sensor{j})(:,5) = subject.(fn).(sensor{j})(:,6);
            reorient.(fn).(sensor{j})(:,6) = -subject.(fn).(sensor{j})(:,5);
            reorient.(fn).(sensor{j})(:,7) = subject.(fn).(sensor{j})(:,7);
        end
    end
    fprintf("Saving Subject Orient Data\n")
    savePath = fullfile(reorientFolderPath, [char(fn) '.mat']);
    save(savePath, '-struct', 'reorient','-v7.3');        
    clear subject reorient
end















