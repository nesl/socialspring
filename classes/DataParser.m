classdef DataParser
    %DATAPARSER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        input_file;
        accel
        grav
        gyro
        accel_lin
        mag
        euler
        gps
        displacement
        speed
        heading
    end
    
    methods
        
        function obj = DataParser(input_file)
            obj.input_file = input_file;
            % parse them data shits
            [accel, grav, gyro, accel_lin,...
                mag, euler, gps, displacement,...
                speed, heading] = parseRawData(input_file)
            
            % assign object arrays
            obj.accel = accel;
            obj.grav = grav;
            obj.gyro = gyro;
            obj.accel_lin = accel_lin;
            obj.mag = mag;
            obj.euler = euler;
            obj.gps = gps;
            obj.displacement = displacement;
            obj.speed = speed;
            obj.heading = heading;
            
        end
        
        function [estx, esty] = getDisplacementEstimates(obj)
            estx = [];
            esty = [];
        end
       
    end
    
end

