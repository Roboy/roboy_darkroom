%% read the relative sensor locations from yaml
clear all
config = ReadYaml('calibration.yaml');
rel_pos = cell2mat(config.sensor_relative_locations(:,2:end));

%% pose estimation
for i=1:100
    close all
    % config 3d plot
    h.figure1=figure('Name','darkroom');  
    h.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], ...
              'XGrid','on','YGrid','on','ZGrid','on', ...
              'XLim',[-0.5 0.5],'YLim',[-1 0.2],'ZLim',[-0.5 0.5]);
    view(0,0);
    
	lighthouse_pose = eye(4);
    % this is lighthouse to world pose
    lighthouse_pose(2,4) = 1

    plotCoordinateFrame(inv(lighthouse_pose),0.2,'lighthouse1')

    % generate random object pose
    phi   = rand()*2*pi;
    theta = rand()*pi;
    psi   = rand()*2*pi;

    object_pose = [cos(psi) * cos(phi) - cos(theta) * sin(phi) * sin(psi), cos(psi) * sin(phi) + cos(theta) * cos(phi) * sin(psi), sin(psi) * sin(theta), 0;
        -sin(psi) * cos(phi) - cos(theta) * sin(phi) * cos(psi), -sin(psi) * sin(phi) + cos(theta) * cos(phi) * cos(psi), cos(psi) * sin(theta), 0;
        sin(theta) * sin(phi), -sin(theta) * cos(phi), cos(theta), 0;
        0, 0, 0, 1];

    plotCoordinateFrame(object_pose,0.2,'object')
    plotLighthouseSensors(rel_pos,object_pose,8)

    true_pose = lighthouse_pose*object_pose;

    [elevations, azimuths] = calculateLighthouseAngles(rel_pos,true_pose);
    angles = [elevations azimuths];

    fun = @(x) poseMultiLighthouse(x, rel_pos, angles, lighthouse_pose) ;
    x0 = [0,0,0,0,0,0];
    options = optimset('Display','off');
    x = fsolve(fun,x0,options);
    object_pose_estimate = createRTfrom(x);
    error = norm(object_pose_estimate-object_pose);
    str = ['error in pose estimate ' num2str(error)];
    disp(str)
    if(error<0.00001)
        object_pose
        object_pose_estimate
        plotCoordinateFrame(object_pose_estimate,0.2,'object estimate')
    end
    pause;
end

