%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     resampleCWA: resamples data to 100Hz, for some
%     reason, the IMU repeat datapoints. This is mentioned
%     on their github. Resampling removes replicates.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    % Save resampled data to the 'resample' folder
    savePath = fullfile(resampleFolderPath, [subjectName '.mat']);
    save(savePath, '-struct', 'subject','-v7.3');        
    clear subject
end