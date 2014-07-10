function [accel, grav, gyro, accel_lin, mag, euler, gps, displacement, speed, heading] = parseRawData(filename)

%% Sensor Parsing Constructs
% sensor indices (arbitrary but sequential)
accel_idx = 1;
grav_idx = 2;
gyro_idx = 3;
accel_lin_idx = 4;
mag_idx = 5;
gps_idx = 6;
euler_idx = 7;

% matches the above indices (defined in Android source)
SENSOR_TYPES = [1 9 4 10 2 100 101];
% number of axes per sensor, matching the above indices
SENSOR_AXES = [3 3 3 3 3 2 3];
% amount of sensor data read in so far (starting from 0) corresponding to
% above indices
sensor_counts = [0 0 0 0 0 0 0];


%% Load CSV file (each row has different # of cols)
%filename = 'data/RideKeeper_13_10_22_drivingTest_ucla_shermanoaks';
fprintf('Opening input file...');
fid = fopen(filename);

if fid==-1
    error('could not open file');
end
fprintf('DONE\n');

% get file size
numRows = linecount(fid);
fprintf('getting ready to read %d lines\n', numRows);

% reset fid
fclose(fid);
fid = fopen(filename);

% preallocated vectors with more than enough room for sensor data
accel = zeros(ceil(numRows*0.2),SENSOR_AXES(accel_idx)+1); % # axes + time
grav = zeros(ceil(numRows*0.2),SENSOR_AXES(grav_idx)+1);
gyro = zeros(ceil(numRows*0.2),SENSOR_AXES(gyro_idx)+1);
accel_lin = zeros(ceil(numRows)*0.2,SENSOR_AXES(accel_lin_idx)+1);
mag = zeros(ceil(numRows*0.2),SENSOR_AXES(mag_idx)+1);
gps = zeros(ceil(numRows*0.005),SENSOR_AXES(gps_idx)+1);
euler = zeros(ceil(numRows*0.2),SENSOR_AXES(euler_idx)+1);

line = fgetl(fid);
lineNumber = 1;

print_skip = 1000;
print_counter = print_skip;

while ~feof(fid)
    % display progress
    if print_counter <= 0
        fprintf('%.2f%% parsing complete\n', 100*lineNumber/numRows);
        print_counter = print_skip;
    else
        print_counter = print_counter - 1;
    end
    
    % extract data
    values = strsplit(line,',');
    sensorType = str2double(values{1});
    time = str2double(values{2});
    
    %switch sensorType
    %    case SENSOR_TYPES[accel_idx]
    %        accel = [accel;
    
    if(sensorType == SENSOR_TYPES(accel_idx))
        sensor_counts(accel_idx) = sensor_counts(accel_idx) + 1;
        accel(sensor_counts(accel_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    elseif(sensorType == SENSOR_TYPES(grav_idx))
        sensor_counts(grav_idx) = sensor_counts(grav_idx) + 1;
        grav(sensor_counts(grav_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    end
    
    if(sensorType == SENSOR_TYPES(gyro_idx))
        sensor_counts(gyro_idx) = sensor_counts(gyro_idx) + 1;
        gyro(sensor_counts(gyro_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    end
    
    if(sensorType == SENSOR_TYPES(accel_lin_idx))
        sensor_counts(accel_lin_idx) = sensor_counts(accel_lin_idx) + 1;
        accel_lin(sensor_counts(accel_lin_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    end
    
    if(sensorType == SENSOR_TYPES(mag_idx))
        sensor_counts(mag_idx) = sensor_counts(mag_idx) + 1;
        mag(sensor_counts(mag_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    end
    
    if(sensorType == SENSOR_TYPES(gps_idx))
        sensor_counts(gps_idx) = sensor_counts(gps_idx) + 1;
        gps(sensor_counts(gps_idx),:) = [
            time str2double(values(4)) str2double(values(3))
            ];
    end
    
    if(sensorType == SENSOR_TYPES(euler_idx))
        sensor_counts(euler_idx) = sensor_counts(euler_idx) + 1;
        euler(sensor_counts(euler_idx),:) = [
            time str2double(values(3)) str2double(values(4)) str2double(values(5))
            ];
    end

    % get next line
    line = fgetl(fid);
    lineNumber = lineNumber + 1;
end

fclose(fid);

%% Prune unused array indices
accel = accel(1:sensor_counts(accel_idx),:);
grav = grav(1:sensor_counts(grav_idx),:);
gyro = gyro(1:sensor_counts(gyro_idx),:);
accel_lin = accel_lin(1:sensor_counts(accel_lin_idx),:);
mag = mag(1:sensor_counts(mag_idx),:);
gps = gps(1:sensor_counts(gps_idx),:);
euler = euler(1:sensor_counts(euler_idx),:);

%% Infer GPS orientation and speed
speed = zeros(size(gps,1),2);
heading = zeros(size(gps,1),2);
displacement = zeros(size(gps,1),3);
conversionToKm = 1.112/0.01;
origin = [gps(1,2) gps(1,3)];

% median filter of gps to throw out erroneous data
gps(:,2) = medfilt1(gps(:,2));
gps(:,3) = medfilt1(gps(:,3));


% initial times
speed(1,1) = gps(1,1);
heading(1,1) = gps(1,1);
displacement(1,1) = gps(1,1);

for i=2:size(gps,1)
    % change in time between gps samples
    dT = ( gps(i,1)-gps(i-1,1) )/1e3; % ms to sec
    time_now = gps(i,1);
    % update displacement
    x = (gps(i,2) - origin(1))*conversionToKm*1000;
    y = (gps(i,3) - origin(2))*conversionToKm*1000;
    displacement(i,:) = [time_now x y];
    % approximate (Euclidean) distance between gps samples
    dX = displacement(i,2)-displacement(i-1,2);
    dY = displacement(i,3)-displacement(i-1,3);
    dist = sqrt( dX^2 + dY^2 );
    % update speed array
    speed(i,:) = [time_now dist/dT];
    % update orientation
    vec = [dX dY]./norm([dX dY]);
    if isnan(vec) % we didn't move
        heading(i,:) = [time_now heading(i-1,2)];
    else
        heading(i,:) = [time_now atan2d(vec(1),vec(2))];
    end
end