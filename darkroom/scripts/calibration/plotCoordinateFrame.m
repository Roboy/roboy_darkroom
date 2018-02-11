function plotCoordinateFrame(pose, scale, name)
%plotCoordinateFrame 
x = [scale 0 0 ;
    0 scale 0 ;
    0 0 scale ;
    1 1 1 ];
X = pose*x;
hold on
plot3([pose(1,4) X(1,1)],[pose(2,4) X(2,1)],[pose(3,4) X(3,1)],'r');
plot3([pose(1,4) X(1,2)],[pose(2,4) X(2,2)],[pose(3,4) X(3,2)],'g');
plot3([pose(1,4) X(1,3)],[pose(2,4) X(2,3)],[pose(3,4) X(3,3)],'b');
text(pose(1,4),pose(2,4),pose(3,4),name);
end

