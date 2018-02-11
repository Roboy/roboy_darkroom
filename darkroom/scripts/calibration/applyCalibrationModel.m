function [elevation_calib, azimuth_calib] = applyCalibrationModel(elevation, azimuth, calib)
%applyCalibrationModel applies calibration model to angles
elevation_calib = elevation;
azimuth_calib = azimuth;
for i=1:length(elevation)
    elevation_calib(i) = elevation_calib(i) + calib(1) + calib(3)*((azimuth_calib(i)-pi/2)/pi)^2 + calib(5)*cos(elevation_calib(i)+calib(7));
    azimuth_calib(i) = azimuth_calib(i) + calib(2) + calib(4)*((elevation_calib(i)-pi/2)/pi)^2 + calib(6)*cos(azimuth_calib(i)+calib(8));
end
end

