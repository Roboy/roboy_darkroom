close all
clear 
clc

MLPEonly = true
step = 10;

g.figure1=figure('Name','compare steamVR and custom tracking');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], 'XGrid','on','YGrid','on','ZGrid','on');   %, ...
          %, ...
          %'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]
          
view(40,30)
hold on
xlabel('x')
ylabel('y')
zlabel('z')
%%
if(MLPEonly)
    X_MLPE = csvread('record_MLPE_calibration_x.log',1);
    mX_MLPE = X_MLPE(1,2:4)-X_MLPE(1,12:14);
    plot3(X_MLPE(:,12)+mX_MLPE(1),X_MLPE(:,13)+mX_MLPE(2),X_MLPE(:,14)+mX_MLPE(3),'m')
    f = plot3(X_MLPE(:,2),X_MLPE(:,3),X_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('VIVE','MLPE')
else
    X_TPE = csvread('record_TPE_calibration_x.log',1);
    mX_TPE = X_TPE(1,2:4)-X_TPE(1,12:14);
    X_MLPE = csvread('record_MLPE_calibration_x.log',1);
    mX_MLPE = X_MLPE(1,2:4)-X_MLPE(1,12:14);
    f = plot3(X_TPE(:,2),X_TPE(:,3),X_TPE(:,4),'b');
    set(f,'LineWidth',2);
    plot3(X_TPE(:,12)+mX_TPE(1),X_TPE(:,13)+mX_TPE(2),X_TPE(:,14)+mX_TPE(3),'m')
    f = plot3(X_MLPE(:,2),X_MLPE(:,3),X_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('TPE','VIVE','MLPE')
end
%%
if(MLPEonly)
    Y_MLPE = csvread('record_MLPE_calibration_y.log',1);
    mY_MLPE = Y_MLPE(1,2:4)-Y_MLPE(1,12:14);
    plot3(Y_MLPE(:,12)+mY_MLPE(1),Y_MLPE(:,13)+mY_MLPE(2),Y_MLPE(:,14)+mY_MLPE(3),'m')
    f = plot3(Y_MLPE(:,2),Y_MLPE(:,3),Y_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('VIVE','MLPE')
else
    Y_TPE = csvread('record_TPE_calibration_y.log',1);
    mY_TPE = Y_TPE(1,2:4)-Y_TPE(1,12:14);
    Y_MLPE = csvread('record_MLPE_calibration_y.log',1);
    mY_MLPE = Y_MLPE(1,2:4)-Y_MLPE(1,12:14);
    f = plot3(Y_TPE(:,2),Y_TPE(:,3),Y_TPE(:,4),'b');
    set(f,'LineWidth',2);
    plot3(Y_TPE(:,12)+mY_TPE(1),Y_TPE(:,13)+mY_TPE(2),Y_TPE(:,14)+mY_TPE(3),'m')
    f = plot3(Y_MLPE(:,2),Y_MLPE(:,3),Y_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('TPE','VIVE','MLPE')
end
%%
if(MLPEonly)
    Z_MLPE = csvread('record_MLPE_calibration_z.log',1);
    mZ_MLPE = Z_MLPE(1,2:4)-Z_MLPE(1,12:14);
    plot3(Z_MLPE(:,12)+mZ_MLPE(1),Z_MLPE(:,13)+mZ_MLPE(2),Z_MLPE(:,14)+mZ_MLPE(3),'m')
    f = plot3(Z_MLPE(:,2),Z_MLPE(:,3),Z_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('VIVE','MLPE')
else
    Z_TPE = csvread('record_TPE_calibration_z.log',1);
    mZ_TPE = Z_TPE(1,2:4)-Z_TPE(1,12:14);
    Z_MLPE = csvread('record_MLPE_calibration_z.log',1);
    mZ_MLPE = Z_MLPE(1,2:4)-Z_MLPE(1,12:14);
    f = plot3(Z_TPE(:,2),Z_TPE(:,3),Z_TPE(:,4),'b');
    set(f,'LineWidth',2);
    plot3(Z_TPE(:,12)+mZ_TPE(1),Z_TPE(:,13)+mZ_TPE(2),Z_TPE(:,14)+mZ_TPE(3),'m')
    f = plot3(Z_MLPE(:,2),Z_MLPE(:,3),Z_MLPE(:,4),'g');
    set(f,'LineWidth',2);
    legend('TPE','VIVE','MLPE')
end
%%
figure(3)
clf
hold on
plot(X_TPE(1:end,12)+mX_TPE(1),X_TPE(1:end,13)+mX_TPE(2),'m')
plot(X_MLPE(1:end,2),X_MLPE(1:end,3),'g')
plot(X_TPE(1:end,2),X_TPE(1:end,3),'b')
xlabel('x[m]')
ylabel('y[m]')
axis equal
legend('VIVE','MLPE','TPE')
matlab2tikz('x_calibratedComparisonXY.tex','width','\fwidth','height','\fheight');

tX_MLPE = (X_MLPE(:,1) - X_MLPE(1,1))./1000000; % convert to milliseconds
tX_TPE = (X_TPE(:,1) - X_TPE(1,1))./1000000; % convert to milliseconds
vel = sqrt(X_TPE(:,15).^2+X_TPE(:,16).^2+X_TPE(:,17).^2);
indices = vel~=0;
figure(4)
clf
hold on
plot(tX_TPE(indices,1),vel(indices) ,'m')
poserrorMLPE = sqrt((X_MLPE(:,2)-(X_MLPE(:,12)+mX_MLPE(1))).^2+...
    (X_MLPE(:,3)-(X_MLPE(:,13)+mX_MLPE(2))).^2+...
    (X_MLPE(:,4)-(X_MLPE(:,14)+mX_MLPE(3))).^2);
axis([0,tX_TPE(end),min(vel),max(vel)])
xlabel('t[ms]')
ylabel('velocity[m/s]')
legend('Vive')
matlab2tikz('x_calibratedComparisonVelocity.tex','width','\fwidth','height','\fheight');
figure(5)
clf
hold on
plot(tX_MLPE(:,1), poserrorMLPE,'g')
poserrorTPE = sqrt((X_TPE(indices,2)-(X_TPE(indices,12)+mX_TPE(1))).^2+...
    (X_TPE(indices,3)-(X_TPE(indices,13)+mX_TPE(2))).^2+...
    (X_TPE(indices,4)-(X_TPE(indices,14)+mX_TPE(3))).^2);
plot(tX_TPE(indices,1), poserrorTPE,'b')
axis([0,tX_TPE(end),...
    min([poserrorTPE;poserrorMLPE]),max([poserrorTPE;poserrorMLPE])])
xlabel('t[ms]')
ylabel('MSE[m]')
legend('MLPE','TPE')
matlab2tikz('x_calibratedComparisonPositionError.tex','width','\fwidth','height','\fheight');
%%
figure(6)
clf
hold on
plot(Y_TPE(1:end,12)+mY_TPE(1),Y_TPE(1:end,13)+mY_TPE(2),'m-')
plot(Y_MLPE(1:end,2),Y_MLPE(1:end,3),'g')
plot(Y_TPE(1:end,2),Y_TPE(1:end,3),'b')
xlabel('x[m]')
ylabel('y[m]')
axis equal
legend('VIVE','MLPE','TPE')
matlab2tikz('y_calibratedComparisonXY.tex','width','\fwidth','height','\fheight');

tY_MLPE = (Y_MLPE(:,1) - Y_MLPE(1,1))./1000000; % convert to milliseconds
tY_TPE = (Y_TPE(:,1) - Y_TPE(1,1))./1000000; % convert to milliseconds
vel = sqrt(Y_TPE(:,15).^2+Y_TPE(:,16).^2+Y_TPE(:,17).^2);
indices = vel~=0;
figure(7)
clf
hold on
plot(tY_TPE(indices,1),vel(indices) ,'m')
poserrorMLPE = sqrt((Y_MLPE(:,2)-(Y_MLPE(:,12)+mY_MLPE(1))).^2+...
    (Y_MLPE(:,3)-(Y_MLPE(:,13)+mY_MLPE(2))).^2+...
    (Y_MLPE(:,4)-(Y_MLPE(:,14)+mY_MLPE(3))).^2);
axis([0,tY_TPE(end),min(vel),max(vel)])
xlabel('t[ms]')
ylabel('velocity[m/s]')
legend('Vive')
matlab2tikz('y_calibratedComparisonVelocity.tex','width','\fwidth','height','\fheight');
figure(8)
clf
hold on
plot(tY_MLPE(:,1), poserrorMLPE,'g')
poserrorTPE = sqrt((Y_TPE(indices,2)-(Y_TPE(indices,12)+mY_TPE(1))).^2+...
    (Y_TPE(indices,3)-(Y_TPE(indices,13)+mY_TPE(2))).^2+...
    (Y_TPE(indices,4)-(Y_TPE(indices,14)+mY_TPE(3))).^2);
plot(tY_TPE(indices,1), poserrorTPE,'b')
axis([0,tY_TPE(end),...
    min([poserrorTPE;poserrorMLPE]),max([poserrorTPE;poserrorMLPE])])
xlabel('t[ms]')
ylabel('MSE[m]')
legend('MLPE','TPE')
matlab2tikz('y_calibratedComparisonPositionError.tex','width','\fwidth','height','\fheight');
%%
figure(9)
clf
hold on
plot(Z_TPE(1:end,12)+mZ_TPE(1),Z_TPE(1:end,14)+mZ_TPE(3),'m-')
plot(Z_MLPE(1:end,2),Z_MLPE(1:end,4),'g')
plot(Z_TPE(1:end,2),Z_TPE(1:end,4),'b')
xlabel('x[m]')
ylabel('z[m]')
axis equal
legend('VIVE','MLPE','TPE')
matlab2tikz('z_calibratedComparisonXZ.tex','width','\fwidth','height','\fheight');

tZ_MLPE = (Z_MLPE(:,1) - Z_MLPE(1,1))./1000000; % convert to milliseconds
tZ_TPE = (Z_TPE(:,1) - Z_TPE(1,1))./1000000; % convert to milliseconds
vel = sqrt(Z_TPE(:,15).^2+Z_TPE(:,16).^2+Z_TPE(:,17).^2);
indices = vel~=0;
figure(10)
clf
hold on
plot(tZ_TPE(indices,1),vel(indices) ,'m')
poserrorMLPE = sqrt((Z_MLPE(:,2)-(Z_MLPE(:,12)+mZ_MLPE(1))).^2+...
    (Z_MLPE(:,3)-(Z_MLPE(:,13)+mZ_MLPE(2))).^2+...
    (Z_MLPE(:,4)-(Z_MLPE(:,14)+mZ_MLPE(3))).^2);
axis([0,tZ_TPE(end),min(vel),max(vel)])
xlabel('t[ms]')
ylabel('velocity[m/s]')
legend('Vive')
matlab2tikz('z_calibratedComparisonVelocity.tex','width','\fwidth','height','\fheight');
figure(11)
clf
hold on
plot(tZ_MLPE(:,1), poserrorMLPE,'g')
poserrorTPE = sqrt((Z_TPE(indices,2)-(Z_TPE(indices,12)+mZ_TPE(1))).^2+...
    (Z_TPE(indices,3)-(Z_TPE(indices,13)+mZ_TPE(2))).^2+...
    (Z_TPE(indices,4)-(Z_TPE(indices,14)+mZ_TPE(3))).^2);
plot(tZ_TPE(indices,1), poserrorTPE,'b')
axis([0,tZ_TPE(end),...
    min([poserrorTPE;poserrorMLPE]),max([poserrorTPE;poserrorMLPE])])
xlabel('t[ms]')
ylabel('MSE[m]')
legend('MLPE','TPE')
matlab2tikz('z_calibratedComparisonPositionError.tex','width','\fwidth','height','\fheight');