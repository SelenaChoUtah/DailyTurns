function [A_data,Rot_data] = rotateIMU(data,order,Fc,Fs)
    % Filters[data] data using butterworth filter with specified [order] order,
    % [Fc] cuttoff frequency and [Fs] sampling frequency.
    [b,a] = butter(order/2,(Fc/(Fs/2)));
    Sway_data = filtfilt(b,a,data)./9.81;
    Sway_v = ((Sway_data(:,1)).*-1);
    Sway_ml = (Sway_data(:,2));
    Sway_ap = (Sway_data(:,3));
    
    % Filtered data is corrected to original coordinate system and averaged
    % across each plane, then substracted from it's corrected data to reach
    % [0,0].
    True_acc_ap = Sway_ap.*(cos(asin(mean(Sway_ap))))- (Sway_v).*(mean(Sway_ap));
    True_acc_vp = Sway_ap.*(mean(Sway_ap))+ (Sway_v).*(cos(asin(mean(Sway_ap))));
    True_acc_ml = Sway_ml.*(cos(asin(mean(Sway_ml))))- (True_acc_vp).*(mean(Sway_ml));
    True_acc_v = Sway_ml.*(mean(Sway_ml))+(True_acc_vp).*(cos(asin(mean(Sway_ml))))-1;
    S_v = mean(True_acc_v);
    S_ml = mean(True_acc_ml);
    S_ap = mean(True_acc_ap);
    AP = True_acc_ap - S_ap;
    ML = True_acc_ml - S_ml;
    Vert = True_acc_v - S_v;
    A_data = [AP ML Vert].*9.81;
    Rot_data = [mean(Sway_ap) mean(Sway_ml)];

end