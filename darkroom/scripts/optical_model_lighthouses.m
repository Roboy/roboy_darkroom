clear
origin_1 = [-0.1 0 0]
origin_2 = [0 0 -0.1]

pos = [];
positions_motor_1 = [];
positions_motor_2 = [];

elevations = [];
azimuths = [];

elevations_motor = [];
azimuths_motor = [];

number_of_sensors = 20

for i=1:number_of_sensors
    v = [rand-0.5 rand rand-0.5];
    pos = [pos;v];
    pos_motor_1 = v - (origin_1);
    pos_motor_2 = v - (origin_2);
    
    positions_motor_1 = [positions_motor_1;pos_motor_1];
    positions_motor_2 = [positions_motor_2;pos_motor_2];
    
    %distance = sqrt(pos_motor_1(1)^2 + pos_motor_1(2)^2 + pos_motor_1(3)^2);
    elevation_motor = pi - atan2(pos_motor_1(2), pos_motor_1(3));
    elevations_motor = [elevations_motor;elevation_motor];
    
    azimuth_motor = atan2(pos_motor_2(2), pos_motor_2(1));
    azimuths_motor = [azimuths_motor;azimuth_motor];
    
    distance = sqrt(v(1)^2 + v(2)^2 + v(3)^2);
    elevation = pi-acos(v(3) / distance);
    azimuth = atan2(v(2), v(1));
    elevations = [elevations;elevation];
    azimuths = [azimuths;azimuth];
end
pos_1 = pos;
pos_2 = pos;
rays_1 = pos_1-origin_1;
rays_2 = pos_2-origin_2;
close all
figure(1)
hold on
j = 1;
for i=1:number_of_sensors
    plot3([0 pos_1(j,1)],[0 pos_1(j,2)],[0 pos_1(j,3)],'k--');
    elevation = pi-elevations(j);
    elevation_motor = elevations_motor(j);
    azimuth = azimuths(j);
    azimuth_motor = azimuths_motor(j);
    
    v_spherical = [sin(elevation)*cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation)];
    plot3([0 v_spherical(1)],[0 v_spherical(2)],[0 v_spherical(3)],'k--');
    %plot3([origin_1(1) pos_1(j,1)],[origin_1(2) pos_1(j,2)],[origin_1(3) pos_1(j,3)],'b--');
    %plot3([origin_2(1) pos_2(j,1)],[origin_2(2) pos_2(j,2)],[origin_2(3) pos_2(j,3)],'r--');
    
    v = [0, sin(elevation_motor), -cos(elevation_motor)];
    h = [cos(azimuth_motor), sin(azimuth_motor), 0];
        
    %plot3([0 v(1)],[0 v(2)],[0 v(3)],'b-');
    %plot3([0 h(1)],[0 h(2)],[0 h(3)],'r-');
    nv = cross(v,[1,0,0]);
    nh = cross(h,[0,0,1]);
    
    ray=cross(nh,nv);
    ray = ray/norm(ray)
    P=[nv;nh;ray]\[dot(nv,origin_1);dot(nh,origin_2);0];
    assert(norm(ray-v_spherical)<0.000001)
    plot3([0 ray(1)],[0 ray(2)],[0 ray(3)],'g-');
    
    j = j+1;
end
axis([-1,1,-1,1,-1,1])
