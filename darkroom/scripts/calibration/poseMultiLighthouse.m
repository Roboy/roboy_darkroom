function F = poseMultiLighthouse(x, rel_pos, angles, lighthouse_pose)
    RT = createRTfrom(x);
    rt = [RT(1,1), RT(2,1), RT(3,1), RT(1,2), RT(2,2), RT(3,2), RT(1,3), RT(2,3), RT(3,3), RT(1,4), RT(2,4), RT(3,4)];
    CD = zeros(2*length(rel_pos),12);
    b = zeros(2*length(rel_pos),1);
    for sensor=1:length(rel_pos)
        elevation = angles(sensor,1);
        azimuth = angles(sensor,2);
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