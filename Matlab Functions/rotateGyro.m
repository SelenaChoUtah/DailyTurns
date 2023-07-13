function [G_data] = rotateGyro(data,Rot_data,order,Fc,Fs)
    % Filters[data] data using butterworth filter with specified [order] order,
    % [Fc] cuttoff frequency and [Fs] sampling frequency. [rData] is the
    % rotation angles found by rotateIMU using the accelerometer.
    [b,a] = butter(order/2,(Fc/(Fs/2)));
    Sway_data = filtfilt(b,a,data);
    Sway_v = ((Sway_data(:,1)));
    Sway_ml = (Sway_data(:,2));
    Sway_ap = (Sway_data(:,3));
    % Filtered data is corrected to original coordinate system and averaged
    % across each plane, then substracted from it's corrected data to reach
    % [0,0].
    True_gyro_ap = Sway_ap.*(cos(Rot_data(:,1)))- (Sway_v).*(Rot_data(:,1));
    True_gyro_vp = Sway_ap.*(Rot_data(:,1))+ (Sway_v).*(cos(Rot_data(:,1)));
    True_gyro_ml = Sway_ml.*(cos(Rot_data(:,2)))- (True_gyro_vp).*(Rot_data(:,2));
    True_gyro_v = Sway_ml.*(Rot_data(:,2))+(True_gyro_vp).*(cos(Rot_data(:,2)));
    S_v = mean(True_gyro_v);
    S_ml = mean(True_gyro_ml);
    S_ap = mean(True_gyro_ap);
    AP = True_gyro_ap - S_ap;
    ML = True_gyro_ml - S_ml;
    Vert = True_gyro_v - S_v;
    G_data = [AP ML Vert];

end