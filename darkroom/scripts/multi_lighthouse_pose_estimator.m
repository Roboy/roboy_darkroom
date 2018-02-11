%% load the data and parse into per sensor per lighthouse
clear
M = csvread('record_simulated_lighthouse0_calibration.log',1);
[sensor_entries,asort] = sort(M(:,2));
b = M(asort,2:end);
s = unique(sensor_entries);
global sensors
sensors = cell(2,length(s));
for i=1:length(s)
    temp = b(sensor_entries==i-1,:);
    [~,lighthouse_sort] = sort(temp(:,2));
    temp = temp(lighthouse_sort,:);
    sensors{1,i} = temp(temp(:,2)==0,3:end);
    sensors{2,i} = temp(temp(:,2)==1,3:end);
end
%% read the relative sensor locations from yaml
config = ReadYaml('calibration.yaml');
global rel_pos 
rel_pos = cell2mat(config.sensor_relative_locations(:,2:end));
%% pose estimation
global sample
sample = 2
true_pose = sensors{1,sample*2}(1,5:end);
true_pose = [true_pose(1:4);true_pose(5:8);true_pose(9:12)];
% for sensor=1:length(rel_pos)
%     if(sensors{1,sensor}(sample,1)==0)
%         azimuth = sensors{1,sensor}(sample,3);
%         elevation = sensors{1,sensor}(sample+1,3);
%     else 
%         azimuth = sensors{1,sensor}(sample+1,3);
%         elevation = sensors{1,sensor}(sample,3);
%     end
%     disp(sensor-1)
%     disp(elevation)
%     disp(azimuth)
% end

global lighthouse_pose
lighthouse_pose = eye(4);
% this is lighthouse to world pose
lighthouse_pose(2,4) = 1

fun = @poseMultiLighthouse;
x0 = [0,0,0,0,0,0];
options = optimset('Display','off');
x = fsolve(fun,x0,options);
RT = createRTfrom(x);
assert(norm(RT-true_pose)<0.0001)
true_pose
RT

function RT = createRTfrom(x)
    alpha_squared = (x(1)^2 + x(2)^2 + x(3)^2)^2;
    q = [(1-alpha_squared)/(alpha_squared+1), 2*x(1)/(alpha_squared+1), 2*x(2)/(alpha_squared+1), 2*x(3)/(alpha_squared+1)];
    q = q/norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    tx = 2*qx;
    ty = 2*qy;
    tz = 2*qz;
    twx = tx*qw;
    twy = ty*qw;
    twz = tz*qw;
    txx = tx*qx;
    txy = ty*qx;
    txz = tz*qx;
    tyy = ty*qy;
    tyz = tz*qy;
    tzz = tz*qz;

    RT = [1-(tyy+tzz),	txy-twz, txz+twy, x(4);
          txy+twz,	1-(txx+tzz), tyz-twx, x(5);
          txz-twy,	tyz+twx , 1-(txx+tyy), x(6)];
end

function F = poseMultiLighthouse(x)
    global rel_pos 
    global sensors
    global lighthouse_pose
    global sample
    RT = createRTfrom(x);

    rt = [RT(1,1), RT(2,1), RT(3,1), RT(1,2), RT(2,2), RT(3,2), RT(1,3), RT(2,3), RT(3,3), RT(1,4), RT(2,4), RT(3,4)];
    CD = zeros(2*length(rel_pos),12);
    b = zeros(2*length(rel_pos),1);
    for sensor=1:length(rel_pos)
        if(sensors{1,sensor}(sample,1)==0)
            azimuth = sensors{1,sensor}(sample,3);
            elevation = sensors{1,sensor}(sample+1,3);
        else 
            azimuth = sensors{1,sensor}(sample+1,3);
            elevation = sensors{1,sensor}(sample,3);
        end
        u = tan(pi/2 - azimuth);
        v = tan(elevation - pi/2);
        C = lighthouse_pose(2,:)*u-lighthouse_pose(1,:);
        D = lighthouse_pose(2,:)*v-lighthouse_pose(3,:);
        X = rel_pos(sensor,1); Y = rel_pos(sensor,2); Z = rel_pos(sensor,3);
        CD(sensor*2:sensor*2+1,:) = [C(1)*X C(2)*X C(3)*X C(1)*Y C(2)*Y C(3)*Y C(1)*Z C(2)*Z C(3)*Z C(1) C(2) C(3);
                            D(1)*X D(2)*X D(3)*X D(1)*Y D(2)*Y D(3)*Y D(1)*Z D(2)*Z D(3)*Z D(1) D(2) D(3)];
        b(sensor*2:sensor*2+1) = [-C(4); -D(4)];
    end
    F = CD*rt' - b;
end