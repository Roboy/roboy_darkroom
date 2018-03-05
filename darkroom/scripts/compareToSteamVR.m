close all
clear 
g.figure1=figure('Name','compare steamVR and custom tracking');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], 'XGrid','on','YGrid','on','ZGrid','on');   %, ...
          %, ...
          %'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]
view(45,45)
hold on
%%
X = csvread('record_calibration_x.log',1);
mX = X(1,2:4)-X(1,12:14);
% mX = [0 0 0];
xlabel('x')
ylabel('y')
zlabel('z')
plot3(X(:,2),X(:,3),X(:,4),'r')
plot3(X(:,12)+mX(1),X(:,13)+mX(2),X(:,14)+mX(3),'r--')
mseX = immse(X(:,2:4),[X(:,12)+mX(1),X(:,13)+mX(2),X(:,14)+mX(3)])
% figure(6)
% plot(sqrt((X(:,2)-X(:,12)).^2+(X(:,3)-X(:,13)).^2+(X(:,4)-X(:,14)).^2));
%%
Y = csvread('record_calibration_y.log',1);
mY = Y(1,2:4)-Y(1,12:14);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
plot3(Y(:,2),Y(:,3),Y(:,4),'g')
plot3(Y(:,12)+mY(1),Y(:,13)+mY(2),Y(:,14)+mY(3),'g--')
mseY = immse(Y(:,2:4),[Y(:,12)+mY(1),Y(:,13)+mY(2),Y(:,14)+mY(3)])
%%
Z = csvread('record_calibration_z.log',1);
mZ = Z(1,2:4)-Z(1,12:14);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
plot3(Z(:,2),Z(:,3),Z(:,4),'b')
plot3(Z(:,12)+mZ(1),Z(:,13)+mZ(2),Z(:,14)+mZ(3),'b--')
mseZ = immse(Z(:,2:4),[Z(:,12)+mZ(1),Z(:,13)+mZ(2),Z(:,14)+mZ(3)])
%%
% h.figure1=figure('Name','distance');  
% plot(A(:,1), sqrt((A(:,2)-A(:,12)).^2+(A(:,3)-A(:,13)).^2+(A(:,4)-A(:,14)).^2))
% %%
% h.figure1=figure('Name','velocity');  
% hold on
% plot(A(:,1), A(:,5),'r')
% plot(A(:,1), A(:,6),'g')
% plot(A(:,1), A(:,7),'b')
%%
% h.figure1=figure('Name','error');  
% hold on
% plot(A(:,1), sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2),'r')
% [C1,lag1] = xcorr(sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2),A(:,6));
% plot(A(:,1),C1(1:2:end,1)/500+mean(sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2)),'g')

%% load data
rotX = csvread('record_calibration_rotX.log',1);
trotX = (rotX(:,1) - rotX(1,1))./1000000; % convert to milliseconds
mX = rotX(1,2:4)-rotX(1,12:14);
rotY = csvread('record_calibration_rotY.log',1);
trotY = (rotY(:,1) - rotY(1,1))./1000000; % convert to milliseconds
mY = rotY(1,2:4)-rotY(1,12:14);
rotZ = csvread('record_calibration_rotZ.log',1);
trotZ = (rotZ(:,1) - rotZ(1,1))./1000000; % convert to milliseconds
mZ = rotZ(1,2:4)-rotZ(1,12:14);
%%
g.figure1=figure('Name','compare steamVR and custom tracking');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], 'XGrid','on','YGrid','on','ZGrid','on');   %, ...
          %, ...
          %'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]
view(45,45)
hold on
xlabel('x')
ylabel('y')
zlabel('z')
plot3(rotX(:,2),rotX(:,3),rotX(:,4),'r')
plot3(rotX(:,12)+mX(1),rotX(:,13)+mX(2),rotX(:,14)+mX(3),'r--')
plot3(rotY(:,2),rotY(:,3),rotY(:,4),'g')
plot3(rotY(:,12)+mY(1),rotY(:,13)+mY(2),rotY(:,14)+mY(3),'g--')
plot3(rotZ(:,2),rotZ(:,3),rotZ(:,4),'b')
plot3(rotZ(:,12)+mZ(1),rotZ(:,13)+mZ(2),rotZ(:,14)+mZ(3),'b--')
%%
[pitch, roll, yaw] = quat2angle([rotX(:,11) rotX(:,8:10)], 'XYZ');
figure(3)
clf
hold on
step = 10;
plot(trotX(1:step:end),pitch(1:step:end),'r')
plot(trotX(1:step:end),roll(1:step:end),'g')
plot(trotX(1:step:end),yaw(1:step:end),'b')
% align the two coordinate frames using the first sample
align = quatdivide([rotX(1,11) rotX(1,8:10)],[rotX(1,21) rotX(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotX(:,21) rotX(:,18:20)],align), 'XYZ');
plot(trotX(1:step:end),pitch_vive(1:step:end),'r--')
plot(trotX(1:step:end),roll_vive(1:step:end),'g--')
plot(trotX(1:step:end),yaw_vive(1:step:end),'b--')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotX(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotX_uncalibratedComparison.tex','width','\fwidth','height','\fheight');
mserotX = immse([pitch roll yaw],[pitch_vive roll_vive yaw_vive])
%%
[pitch, roll, yaw] = quat2angle([rotY(:,11) rotY(:,8:10)], 'XYZ');
figure(4)
clf
hold on
plot(trotY(1:step:end),abs(pitch(1:step:end)),'r')
plot(trotY(1:step:end),roll(1:step:end),'g')
plot(trotY(1:step:end),abs(yaw(1:step:end)),'b')
% align the two coordinate frames using the first sample
align = quatdivide([rotY(1,11) rotY(1,8:10)],[rotY(1,21) rotY(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotY(:,21) rotY(:,18:20)],align), 'XYZ');
plot(trotY(1:step:end),abs(pitch_vive(1:step:end)),'r--')
plot(trotY(1:step:end),roll_vive(1:step:end),'g--')
plot(trotY(1:step:end),abs(yaw_vive(1:step:end)),'b--')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotY(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotY_uncalibratedComparison.tex','width','\fwidth','height','\fheight');
mserotY = immse([abs(pitch) roll abs(yaw)],[abs(pitch_vive) roll_vive abs(yaw_vive)])
%%
[pitch, roll, yaw] = quat2angle([rotZ(:,11) rotZ(:,8:10)], 'XYZ');
figure(5)
clf
hold on
plot(trotZ(1:step:end),pitch(1:step:end),'r')
plot(trotZ(1:step:end),roll(1:step:end),'g')
plot(trotZ(1:step:end),yaw(1:step:end),'b')
% align the two coordinate frames using the first sample
align = quatdivide([rotZ(1,11) rotZ(1,8:10)],[rotZ(1,21) rotZ(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotZ(:,21) rotZ(:,18:20)],align), 'XYZ');
plot(trotZ(1:step:end),pitch_vive(1:step:end),'r--')
plot(trotZ(1:step:end),roll_vive(1:step:end),'g--')
plot(trotZ(1:step:end),yaw_vive(1:step:end),'b--')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotZ(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotZ_uncalibratedComparison.tex','width','\fwidth','height','\fheight');
mserotZ = immse([pitch roll yaw],[pitch_vive roll_vive yaw_vive])