%% read the relative sensor locations from yaml
clear all
close all
config = ReadYaml('calibration.yaml');
rel_pos = cell2mat(config.sensor_relative_locations(:,2:end));
%% pose estimation
lighthouse_pose = eye(4);  
% this is lighthouse to world pose
lighthouse_pose(2,4) = 1

numberOfFrames = 100

plot = false

pose_generator = 'random'
translation_range = 1

object_poses = cell(numberOfFrames,1);
object_pose_estimates = cell(numberOfFrames,1);
angles_true = cell(numberOfFrames,1);
angles_measured = cell(numberOfFrames,1);

calibration_scale = 10
calibration_values = [(rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale (rand()-0.5)/calibration_scale]
% calibration_values = [0 0 0.5 0.4 0 0 0 0];

if(plot)
    g.figure1=figure('Name','simulated calibration values');  
    %config 3d plot
    g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], ...
              'XGrid','on','YGrid','on','ZGrid','on', ...
              'XLim',[-1 1],'YLim',[0 1],'ZLim',[-1 1]);   
    view(0,0)
    hold on
    for i=90:10:90
        elevation = i/180*pi;
        for j=0:10:180
            azimuth = j/180*pi;
            ray = [cos(azimuth)*sin(elevation), sin(azimuth)*sin(elevation), -sin(azimuth)*cos(elevation)];
            ray = ray/norm(ray);
            [ele_calib, azi_calib] = applyCalibrationModel(elevation, azimuth, calibration_values);
            ray_calib = [cos(azi_calib)*sin(ele_calib), sin(azi_calib)*sin(ele_calib), -sin(azi_calib)*cos(ele_calib)];
            ray_calib = ray_calib/norm(ray_calib);
            plot3([0 ray(1)],[0 ray(2)],[0 ray(3)],'g--')
            plot3([0 ray_calib(1)],[0 ray_calib(2)],[0 ray_calib(3)],'r')
        end
    end
end

for frame=1:numberOfFrames    
    object_pose = eye(4);
    switch(pose_generator)
        case 'random'
            % generate random object pose
            phi   = rand()*2*pi*0.1;
            theta = rand()*pi*0.1;
            psi   = rand()*2*pi*0.1;
            x = (rand()-0.5);
            y = (rand()-0.5);
            z = (rand()-0.5);

            object_pose = [cos(psi) * cos(phi) - cos(theta) * sin(phi) * sin(psi), cos(psi) * sin(phi) + cos(theta) * cos(phi) * sin(psi), sin(psi) * sin(theta), x;
                -sin(psi) * cos(phi) - cos(theta) * sin(phi) * cos(psi), -sin(psi) * sin(phi) + cos(theta) * cos(phi) * cos(psi), cos(psi) * sin(theta), y;
                sin(theta) * sin(phi), -sin(theta) * cos(phi), cos(theta), z;
                0, 0, 0, 1];
        case 'rot_x'
            % generate random object pose by rotation around x axis
            phi   = rand()*2*pi;
            object_pose = [1 0 0 0;
                           0 cos(phi) -sin(phi) 0;
                           0 sin(phi) cos(phi) 0;
                           0 0 0 1];
        case 'rot_y'
            phi   = rand()*2*pi;
            object_pose = [cos(phi) 0 sin(phi) 0;
                           0 1 0 0;
                           -sin(phi) 0 cos(phi) 0;
                           0 0 0 1];
        case 'rot_z'
            phi   = rand()*2*pi;
            object_pose = [cos(phi) -sin(phi) 0 0;
                           sin(phi) cos(phi) 0 0;
                           0 0 1 0; 
                           0 0 0 1];
        case 'x'
            x = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [x;0;0];
        case 'y'
            y = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [0;y;0];
        case 'z'
            z = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [0;0;z];
        case 'xy'
            x = (rand()-0.5)*translation_range;
            y = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [x;y;0];
        case 'yz'
            y = (rand()-0.5)*translation_range;
            z = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [0;y;z];
        case 'xz'
            x = (rand()-0.5)*translation_range;
            z = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [x;0;z];
        case 'xyz'
            x = (rand()-0.5)*translation_range;
            y = (rand()-0.5)*translation_range;
            z = (rand()-0.5)*translation_range;
            object_pose(1:3,4) = [x;y;z];
    end

    object_poses{frame} = object_pose;
    
    true_pose = lighthouse_pose*object_pose;

    [elevations_measured,azimuths_measured,elevations_true,azimuths_true] = calculateLighthouseAngles(rel_pos,true_pose,calibration_values);
    ang_true = [elevations_true azimuths_true];
    ang_measured = [elevations_measured azimuths_measured];
    
    angles_true{frame} = ang_true;
    angles_measured{frame} = ang_measured;

    fun = @(x) poseMultiLighthouse(x, rel_pos, ang_measured, lighthouse_pose) ;
    x0 = [0,0,0,0,0,0];
    options = optimset('Display','off');
    x = fsolve(fun,x0,options);
    object_pose_estimate = createRTfrom(x);
    
    object_pose_estimates{frame} = object_pose_estimate;
    
    error = norm(object_pose_estimate-object_pose);

    if(plot)
        %config 3d plot
        h.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], ...
                  'XGrid','on','YGrid','on','ZGrid','on', ...
                  'XLim',[-0.5 0.5],'YLim',[-1 0.2],'ZLim',[-0.5 0.5]);   
        view(0,0)

        plotCoordinateFrame(inv(lighthouse_pose),0.2,'lighthouse1') 
        plotCoordinateFrame(object_pose,0.2,'object')
        plotLighthouseSensors(rel_pos,object_pose,'gx',8)
        plotCoordinateFrame(object_pose_estimate,0.2,'object estimate')
        plotLighthouseSensors(rel_pos,object_pose_estimate,'ro',8)
        
        if(error<0.00001)
            str = ['pose estimation successsful with error ' num2str(error)];
            disp(str)
        else
            str = ['pose estimation failed with error ' num2str(error)];
            disp(str)
        end
        
        object_pose
        object_pose_estimate
        pause;
        clf
    end
    
    disp(frame)

end

%% calibration estimation
calibration_value_estimate = ones(size(calibration_values))*0.0001;
% calibration_value_estimate = [0 0 0.15 0.22 0 0 0 0];

fun = @(x) calibrationEstimator(x, rel_pos, object_poses, lighthouse_pose, angles_measured) ;

options = optimoptions('fsolve');
options.FunctionTolerance = 1.0000e-08;
options.OptimalityTolerance = 1.0000e-08;
calibration_values
calibration_value_estimate = fsolve(fun,calibration_value_estimate,options)
norm(calibration_value_estimate-calibration_values)

angles_estimate = angles_true;
angle_error = zeros(size(angles_true{1}));
for frame=1:numberOfFrames
    [elevation_estimate, azimuth_estimate] = applyCalibrationModel(angles_true{frame}(:,1),angles_true{frame}(:,2),calibration_value_estimate);
    angles_estimate{frame} = [elevation_estimate azimuth_estimate];
    angle_error = angle_error + (angles_estimate{frame}-angles_measured{frame}).^2;
end
disp(angle_error)

function F = calibrationEstimator(x, rel_pos, object_poses, lighthouse_pose, angles_measured)
    F = [0;0];
    for frame=1:length(object_poses)
        [elevations_estimate,azimuths_estimate] = calculateLighthouseAngles(rel_pos,lighthouse_pose*object_poses{frame},x);
        for i=1:length(elevations_estimate)
            F(1) = F(1) + (angles_measured{frame}(i,1)-elevations_estimate(i));
            F(2) = F(2) + (angles_measured{frame}(i,2)-azimuths_estimate(i));
        end
    end
end