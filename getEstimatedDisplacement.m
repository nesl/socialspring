function [ estimate ] = getEstimatedDisplacement( accel_lin, gyro )

%% Phase 1: Turn Detection
% look at z-axis only
gyr = gyro(:,4);
gyr_t = 1e-9*gyro(:,1) - 1e-9*gyro(1,1);
% filter
fc = 0.5;
SR = mean( 1./( diff(gyr_t) ) );
[b,a] = butter(2, 2*fc/SR);
gyr_f = filtfilt(b,a,gyr);
% clip small signals
thresh = 0.1;
gyr_f( find( abs(gyr_f) < thresh ) ) = 0;
% integrate
angle_start = (90);
delta_angle = 1.2*cumsum(gyr_f);

% constrain to [0,360]
delta_angle = mod(angle_start + delta_angle, 360);
% add to initial
angles = [0 angle_start; [gyr_t delta_angle] ];

%% Phase 2: Step Detection
stridelength = 0.610; % meters
% look at z-axis only
acc = accel_lin(:,4);
acc_t = 1e-9*accel_lin(:,1) - 1e-9*accel_lin(1,1);
% filter
fc = 5;
SR = mean( 1./( diff(acc_t) ) );
[b,a] = butter(2, 2*fc/SR);
acc_f = filtfilt(b,a,acc);
% find peaks
[steps,step_locs] = findpeaks(acc_f, 'minpeakheight', 1.0);
% conver to distance vector
distance = [0 0];
for i=1:length(steps)
    t = acc_t(step_locs( i ));
    dist = distance(i, 2) + stridelength;
    distance = [distance; [t dist]];
end

%% Phase 3: Displacement
estimate = [0 0 0]; % time, x, y
for i=2:size(distance,1)
    t = distance(i,1);
    % find most recent angle
    last_angle_idx = find(angles(:,1) < t, 1, 'last');
    last_angle = angles(last_angle_idx, 2);
    % increment the estimate vector
    xy_last = estimate(i-1,2:end);
    delta = distance(i,2) - distance(i-1,2);

    new = [t (xy_last + [delta*cosd(last_angle) delta*sind(last_angle)]) ];
    
    estimate = [estimate; new];
end


end

