function [elevation,azimuth] = calculateLighthouseAngles(rel_pos,pose,varargin)
%calculateLighthouseAngles calculates lighthouse angles from rel_pos and a
% given pose, optionally applying calibration values
AXIS_OFFSET = 0.015;
rel_pos = [rel_pos';ones(size(rel_pos,1),1)'];
pos = pose*rel_pos;
elevation = zeros(size(rel_pos,2),1);
azimuth = zeros(size(rel_pos,2),1);
apply_calibration = false;
if(isempty(varargin)==0)
   calib = varargin{1}; 
   apply_calibration = true;
end
for i=1:length(elevation)
	sensor_pos_motor_vertical = pos(:,i) - [-AXIS_OFFSET,0,0,0]';
    sensor_pos_motor_horizontal = pos(:,i) - [0,0,-AXIS_OFFSET,0]';
    elevation_true = pi -  atan2(sensor_pos_motor_vertical(2), sensor_pos_motor_vertical(3));
   	azimuth_true = atan2(sensor_pos_motor_horizontal(2), sensor_pos_motor_horizontal(1));
    if(apply_calibration)
       
    else
        elevation(i) = elevation_true;
        azimuth(i) = azimuth_true;
    end
end
end

