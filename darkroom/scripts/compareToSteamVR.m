close all
clear 
clc
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
%%
X_TPE = csvread('record_TPE_calibration_x.log',1);
mX_TPE = X_TPE(1,2:4)-X_TPE(1,12:14);
mseX_TPE = immse(X_TPE(:,2:4),[X_TPE(:,12)+mX_TPE(1),X_TPE(:,13)+mX_TPE(2),X_TPE(:,14)+mX_TPE(3)])

plot3(X_TPE(:,2),X_TPE(:,3),X_TPE(:,4),'b')
plot3(X_TPE(:,12)+mX_TPE(1),X_TPE(:,13)+mX_TPE(2),X_TPE(:,14)+mX_TPE(3),'m--')

X_MLPE = csvread('record_MLPE_calibration_x.log',1);
mX_MLPE = X_MLPE(1,2:4)-X_MLPE(1,12:14);
mseX_MLPE = immse(X_MLPE(:,2:4),[X_MLPE(:,12)+mX_MLPE(1),X_MLPE(:,13)+mX_MLPE(2),X_MLPE(:,14)+mX_MLPE(3)])

plot3(X_MLPE(:,2),X_MLPE(:,3),X_MLPE(:,4),'g')
%%
Y_TPE = csvread('record_TPE_calibration_y.log',1);
mY_TPE = Y_TPE(1,2:4)-Y_TPE(1,12:14);
mseY_TPE = immse(Y_TPE(:,2:4),[Y_TPE(:,12)+mY_TPE(1),Y_TPE(:,13)+mY_TPE(2),Y_TPE(:,14)+mY_TPE(3)])

plot3(Y_TPE(:,2),Y_TPE(:,3),Y_TPE(:,4),'b')
plot3(Y_TPE(:,12)+mY_TPE(1),Y_TPE(:,13)+mY_TPE(2),Y_TPE(:,14)+mY_TPE(3),'m--')

Y_MLPE = csvread('record_MLPE_calibration_y.log',1);
mY_MLPE = Y_MLPE(1,2:4)-Y_MLPE(1,12:14);
mseY_MLPE = immse(Y_MLPE(:,2:4),[Y_MLPE(:,12)+mY_MLPE(1),Y_MLPE(:,13)+mY_MLPE(2),Y_MLPE(:,14)+mY_MLPE(3)])

plot3(Y_MLPE(:,2),Y_MLPE(:,3),Y_MLPE(:,4),'g')
%%
Z_TPE = csvread('record_TPE_calibration_z.log',1);
mZ_TPE = Z_TPE(1,2:4)-Z_TPE(1,12:14);
mseY_TPE = immse(Z_TPE(:,2:4),[Z_TPE(:,12)+mZ_TPE(1),Z_TPE(:,13)+mZ_TPE(2),Z_TPE(:,14)+mZ_TPE(3)])

plot3(Z_TPE(:,2),Z_TPE(:,3),Z_TPE(:,4),'b')
plot3(Z_TPE(:,12)+mZ_TPE(1),Z_TPE(:,13)+mZ_TPE(2),Z_TPE(:,14)+mZ_TPE(3),'m--')

Z_MLPE = csvread('record_MLPE_calibration_z.log',1);
mZ_MLPE = Z_MLPE(1,2:4)-Z_MLPE(1,12:14);
mseY_MLPE = immse(Z_MLPE(:,2:4),[Z_MLPE(:,12)+mZ_MLPE(1),Z_MLPE(:,13)+mZ_MLPE(2),Z_MLPE(:,14)+mZ_MLPE(3)])

plot3(Z_MLPE(:,2),Z_MLPE(:,3),Z_MLPE(:,4),'g')
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
rotX_TPE = csvread('record_TPE_calibration_rotX.log',1);
trotX_TPE = (rotX_TPE(:,1) - rotX_TPE(1,1))./1000000; % convert to milliseconds
mX_TPE = rotX_TPE(1,2:4)-rotX_TPE(1,12:14);
rotY_TPE = csvread('record_TPE_calibration_rotY.log',1);
trotY_TPE = (rotY_TPE(:,1) - rotY_TPE(1,1))./1000000; % convert to milliseconds
mY_TPE = rotY_TPE(1,2:4)-rotY_TPE(1,12:14);
rotZ_TPE = csvread('record_TPE_calibration_rotZ.log',1);
trotZ_TPE = (rotZ_TPE(:,1) - rotZ_TPE(1,1))./1000000; % convert to milliseconds
mZ_TPE = rotZ_TPE(1,2:4)-rotZ_TPE(1,12:14);

rotX_MLPE = csvread('record_MLPE_calibration_rotX.log',1);
trotX_MLPE = (rotX_MLPE(:,1) - rotX_MLPE(1,1))./1000000; % convert to milliseconds
mX_TPE = rotX_MLPE(1,2:4)-rotX_MLPE(1,12:14);
rotY_MLPE = csvread('record_MLPE_calibration_rotY.log',1);
trotY_MLPE = (rotY_MLPE(:,1) - rotY_MLPE(1,1))./1000000; % convert to milliseconds
mY_TPE = rotY_MLPE(1,2:4)-rotY_MLPE(1,12:14);
rotZ_MLPE = csvread('record_MLPE_calibration_rotZ.log',1);
trotZ_MLPE = (rotZ_MLPE(:,1) - rotZ_MLPE(1,1))./1000000; % convert to milliseconds
mZ_TPE = rotZ_MLPE(1,2:4)-rotZ_MLPE(1,12:14);
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
plot3(rotX_TPE(:,2),rotX_TPE(:,3),rotX_TPE(:,4),'r')
plot3(rotX_TPE(:,12)+mX_TPE(1),rotX_TPE(:,13)+mX_TPE(2),rotX_TPE(:,14)+mX_TPE(3),'r--')
plot3(rotY_TPE(:,2),rotY_TPE(:,3),rotY_TPE(:,4),'g')
plot3(rotY_TPE(:,12)+mY_TPE(1),rotY_TPE(:,13)+mY_TPE(2),rotY_TPE(:,14)+mY_TPE(3),'g--')
plot3(rotZ_TPE(:,2),rotZ_TPE(:,3),rotZ_TPE(:,4),'b')
plot3(rotZ_TPE(:,12)+mZ_TPE(1),rotZ_TPE(:,13)+mZ_TPE(2),rotZ_TPE(:,14)+mZ_TPE(3),'b--')
%%
[pitch, roll, yaw] = quat2angle([rotX_TPE(:,11) rotX_TPE(:,8:10)], 'XYZ');
mserotX_TPE = immse([pitch roll yaw],[pitch_vive roll_vive yaw_vive])
figure(3)
clf
hold on
step = 10;
plot(trotX_TPE(1:step:end),pitch(1:step:end),'ro')
plot(trotX_TPE(1:step:end),roll(1:step:end),'go')
plot(trotX_TPE(1:step:end),yaw(1:step:end),'bo')
% align the two coordinate frames using the first sample
align = quatdivide([rotX_TPE(1,11) rotX_TPE(1,8:10)],[rotX_TPE(1,21) rotX_TPE(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotX_TPE(:,21) rotX_TPE(:,18:20)],align), 'XYZ');
plot(trotX_TPE(1:step:end),pitch_vive(1:step:end),'r--')
plot(trotX_TPE(1:step:end),roll_vive(1:step:end),'g--')
plot(trotX_TPE(1:step:end),yaw_vive(1:step:end),'b--')

[pitch, roll, yaw] = quat2angle([rotX_MLPE(:,11) rotX_MLPE(:,8:10)], 'XYZ');
mserotX_MLPE = immse([pitch roll yaw],[pitch_vive roll_vive yaw_vive])
plot(trotX_TPE(1:step:end),pitch(1:step:end),'rx')
plot(trotX_TPE(1:step:end),roll(1:step:end),'gx')
plot(trotX_TPE(1:step:end),yaw(1:step:end),'bx')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotX_TPE(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotX_uncalibratedComparison.tex','width','\fwidth','height','\fheight');

%%
[pitch, roll, yaw] = quat2angle([rotY_TPE(:,11) rotY_TPE(:,8:10)], 'XYZ');
figure(4)
clf
hold on
plot(trotY_TPE(1:step:end),abs(pitch(1:step:end)),'r')
plot(trotY_TPE(1:step:end),roll(1:step:end),'g')
plot(trotY_TPE(1:step:end),abs(yaw(1:step:end)),'b')
% align the two coordinate frames using the first sample
align = quatdivide([rotY_TPE(1,11) rotY_TPE(1,8:10)],[rotY_TPE(1,21) rotY_TPE(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotY_TPE(:,21) rotY_TPE(:,18:20)],align), 'XYZ');
plot(trotY_TPE(1:step:end),abs(pitch_vive(1:step:end)),'r--')
plot(trotY_TPE(1:step:end),roll_vive(1:step:end),'g--')
plot(trotY_TPE(1:step:end),abs(yaw_vive(1:step:end)),'b--')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotY_TPE(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotY_uncalibratedComparison.tex','width','\fwidth','height','\fheight');
mserotY = immse([abs(pitch) roll abs(yaw)],[abs(pitch_vive) roll_vive abs(yaw_vive)])
%%
[pitch, roll, yaw] = quat2angle([rotZ_TPE(:,11) rotZ_TPE(:,8:10)], 'XYZ');
figure(5)
clf
hold on
plot(trotZ_TPE(1:step:end),pitch(1:step:end),'r')
plot(trotZ_TPE(1:step:end),roll(1:step:end),'g')
plot(trotZ_TPE(1:step:end),yaw(1:step:end),'b')
% align the two coordinate frames using the first sample
align = quatdivide([rotZ_TPE(1,11) rotZ_TPE(1,8:10)],[rotZ_TPE(1,21) rotZ_TPE(1,18:20)]);
[pitch_vive, roll_vive, yaw_vive] = ...
    quat2angle(quatmultiply([rotZ_TPE(:,21) rotZ_TPE(:,18:20)],align), 'XYZ');
plot(trotZ_TPE(1:step:end),pitch_vive(1:step:end),'r--')
plot(trotZ_TPE(1:step:end),roll_vive(1:step:end),'g--')
plot(trotZ_TPE(1:step:end),yaw_vive(1:step:end),'b--')
xlabel('t[ms]')
ylabel('euler[rad]')
axis([0,trotZ_TPE(end),-pi,pi])
legend('pitch','roll','yaw')
matlab2tikz('rotZ_uncalibratedComparison.tex','width','\fwidth','height','\fheight');
mserotZ = immse([pitch roll yaw],[pitch_vive roll_vive yaw_vive])