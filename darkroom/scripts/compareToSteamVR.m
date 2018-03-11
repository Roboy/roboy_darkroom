close all
clear 
clc

MLPEonly = true
step = 5;

g.figure1=figure('Name','compare steamVR and custom tracking');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], 'XGrid','on','YGrid','on','ZGrid','on');   %, ...
          %, ...
          %'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]
          
view(40,30)
hold on
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')

%% load data
% MLPE
X_MLPE = csvread('record_MLPE_calibration_x.log',1);
Y_MLPE = csvread('record_MLPE_calibration_y.log',1);
Z_MLPE = csvread('record_MLPE_calibration_z.log',1);
mX_MLPE = X_MLPE(1,2:4)-X_MLPE(1,12:14);
mY_MLPE = Y_MLPE(1,2:4)-Y_MLPE(1,12:14);
mZ_MLPE = Z_MLPE(1,2:4)-Z_MLPE(1,12:14);
mX_MLPE_uncalibrated = X_MLPE(1,22:24)-X_MLPE(1,12:14);
mY_MLPE_uncalibrated = Y_MLPE(1,22:24)-Y_MLPE(1,12:14);
mZ_MLPE_uncalibrated = Z_MLPE(1,22:24)-Z_MLPE(1,12:14);
% TPE
if(~MLPEonly)
X_TPE = csvread('record_TPE_calibration_x.log',1);
Y_TPE = csvread('record_TPE_calibration_y.log',1);
Z_TPE = csvread('record_TPE_calibration_z.log',1);
mX_TPE = X_TPE(1,2:4)-X_TPE(1,12:14);
mY_TPE = Y_TPE(1,2:4)-Y_TPE(1,12:14);
mZ_TPE = Z_TPE(1,2:4)-Z_TPE(1,12:14);
mX_TPE_uncalibrated = X_TPE(1,22:24)-X_TPE(1,12:14);
mY_TPE_uncalibrated = Y_TPE(1,22:24)-Y_TPE(1,12:14);
mZ_TPE_uncalibrated = Z_TPE(1,22:24)-Z_TPE(1,12:14);
end
%% RMSE
rmseX_MLPE = sqrt(sum((X_MLPE(:,2:4)-[X_MLPE(:,12)+mX_MLPE(1),X_MLPE(:,13)+mX_MLPE(2),X_MLPE(:,14)+mX_MLPE(3)]).^2)/size(X_MLPE,1));
rmseY_MLPE = sqrt(sum((Y_MLPE(:,2:4)-[Y_MLPE(:,12)+mY_MLPE(1),Y_MLPE(:,13)+mY_MLPE(2),Y_MLPE(:,14)+mY_MLPE(3)]).^2)/size(Y_MLPE,1));
rmseZ_MLPE = sqrt(sum((Z_MLPE(:,2:4)-[Z_MLPE(:,12)+mZ_MLPE(1),Z_MLPE(:,13)+mZ_MLPE(2),Z_MLPE(:,14)+mZ_MLPE(3)]).^2)/size(Z_MLPE,1));

rmseX_MLPE_uncalibrated = sqrt(sum((X_MLPE(:,22:24)-[X_MLPE(:,12)+mX_MLPE_uncalibrated(1),X_MLPE(:,13)+mX_MLPE_uncalibrated(2),X_MLPE(:,14)+mX_MLPE_uncalibrated(3)]).^2)/size(X_MLPE,1));
rmseY_MLPE_uncalibrated = sqrt(sum((Y_MLPE(:,22:24)-[Y_MLPE(:,12)+mY_MLPE_uncalibrated(1),Y_MLPE(:,13)+mY_MLPE_uncalibrated(2),Y_MLPE(:,14)+mY_MLPE_uncalibrated(3)]).^2)/size(Y_MLPE,1));
rmseZ_MLPE_uncalibrated = sqrt(sum((Z_MLPE(:,22:24)-[Z_MLPE(:,12)+mZ_MLPE_uncalibrated(1),Z_MLPE(:,13)+mZ_MLPE_uncalibrated(2),Z_MLPE(:,14)+mZ_MLPE_uncalibrated(3)]).^2)/size(Z_MLPE,1));

rmse_MLPE = rmseX_MLPE + rmseY_MLPE + rmseZ_MLPE
rmse_MLPE_uncalibrated = rmseX_MLPE_uncalibrated + rmseY_MLPE_uncalibrated + rmseZ_MLPE_uncalibrated
if(~MLPEonly)
rmseX_TPE = sqrt(sum((X_TPE(:,2:4)-[X_TPE(:,12)+mX_TPE(1) X_TPE(:,13)+mX_TPE(2) X_TPE(:,14)+mX_TPE(3)]).^2)/size(X_TPE,1));
rmseY_TPE = sqrt(sum((Y_TPE(:,2:4)-[Y_TPE(:,12)+mY_TPE(1),Y_TPE(:,13)+mY_TPE(2),Y_TPE(:,14)+mY_TPE(3)]).^2)/size(Y_TPE,1));
rmseZ_TPE = sqrt(sum((Z_TPE(:,2:4)-[Z_TPE(:,12)+mZ_TPE(1),Z_TPE(:,13)+mZ_TPE(2),Z_TPE(:,14)+mZ_TPE(3)]).^2)/size(Z_TPE,1));

rmseX_TPE_uncalibrated = sqrt(sum((X_TPE(:,22:24)-[X_TPE(:,12)+mX_TPE_uncalibrated(1) X_TPE(:,13)+mX_TPE_uncalibrated(2) X_TPE(:,14)+mX_TPE_uncalibrated(3)]).^2)/size(X_TPE,1));
rmseY_TPE_uncalibrated = sqrt(sum((Y_TPE(:,22:24)-[Y_TPE(:,12)+mY_TPE_uncalibrated(1),Y_TPE(:,13)+mY_TPE_uncalibrated(2),Y_TPE(:,14)+mY_TPE_uncalibrated(3)]).^2)/size(Y_TPE,1));
rmseZ_TPE_uncalibrated = sqrt(sum((Z_TPE(:,22:24)-[Z_TPE(:,12)+mZ_TPE_uncalibrated(1),Z_TPE(:,13)+mZ_TPE_uncalibrated(2),Z_TPE(:,14)+mZ_TPE_uncalibrated(3)]).^2)/size(Z_TPE,1));

rmse_TPE = rmseX_TPE + rmseY_TPE + rmseZ_TPE
rmse_TPE_uncalibrated = rmseX_TPE_uncalibrated + rmseY_TPE_uncalibrated + rmseZ_TPE_uncalibrated
end
%% plot3d
if(MLPEonly)
    % X
    f = plot3(X_MLPE(:,12)+mX_MLPE(1),X_MLPE(:,13)+mX_MLPE(2),X_MLPE(:,14)+mX_MLPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(X_MLPE(:,12)+mX_MLPE_uncalibrated(1),X_MLPE(:,13)+mX_MLPE_uncalibrated(2),X_MLPE(:,14)+mX_MLPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(X_MLPE(:,2),X_MLPE(:,3),X_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(X_MLPE(:,22),X_MLPE(:,23),X_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    % Y
    f = plot3(Y_MLPE(:,12)+mY_MLPE(1),Y_MLPE(:,13)+mY_MLPE(2),Y_MLPE(:,14)+mY_MLPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(Y_MLPE(:,12)+mY_MLPE_uncalibrated(1),Y_MLPE(:,13)+mY_MLPE_uncalibrated(2),Y_MLPE(:,14)+mY_MLPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(Y_MLPE(:,2),Y_MLPE(:,3),Y_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(Y_MLPE(:,22),Y_MLPE(:,23),Y_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    % Z
    f = plot3(Z_MLPE(:,12)+mZ_MLPE(1),Z_MLPE(:,13)+mZ_MLPE(2),Z_MLPE(:,14)+mZ_MLPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(Z_MLPE(:,12)+mZ_MLPE_uncalibrated(1),Z_MLPE(:,13)+mZ_MLPE_uncalibrated(2),Z_MLPE(:,14)+mZ_MLPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(Z_MLPE(:,2),Z_MLPE(:,3),Z_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(Z_MLPE(:,22),Z_MLPE(:,23),Z_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    legend('VIVE', 'VIVE_{alignedToUncalibrated}','MLPE_{calibrated}','MLPE_{uncalibrated}')
else
    % X
    f = plot3(X_TPE(:,12)+mX_TPE(1),X_TPE(:,13)+mX_TPE(2),X_TPE(:,14)+mX_TPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(X_TPE(:,12)+mX_TPE_uncalibrated(1),X_TPE(:,13)+mX_TPE_uncalibrated(2),X_TPE(:,14)+mX_TPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(X_MLPE(:,2),X_MLPE(:,3),X_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(X_MLPE(:,22),X_MLPE(:,23),X_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    f = plot3(X_TPE(:,2),X_TPE(:,3),X_TPE(:,4),'b'); set(f,'LineWidth',2);
    f = plot3(X_TPE(:,22),X_TPE(:,23),X_TPE(:,24),'b--'); set(f,'LineWidth',2);
    % Y
    f = plot3(Y_TPE(:,12)+mY_TPE(1),Y_TPE(:,13)+mY_TPE(2),Y_TPE(:,14)+mY_TPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(Y_TPE(:,12)+mY_TPE_uncalibrated(1),Y_TPE(:,13)+mY_TPE_uncalibrated(2),Y_TPE(:,14)+mY_TPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(Y_MLPE(:,2),Y_MLPE(:,3),Y_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(Y_MLPE(:,22),Y_MLPE(:,23),Y_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    f = plot3(Y_TPE(:,2),Y_TPE(:,3),Y_TPE(:,4),'b'); set(f,'LineWidth',2);
    f = plot3(Y_TPE(:,22),Y_TPE(:,23),Y_TPE(:,24),'b--'); set(f,'LineWidth',2);
    % Z
    f = plot3(Z_TPE(:,12)+mZ_TPE(1),Z_TPE(:,13)+mZ_TPE(2),Z_TPE(:,14)+mZ_TPE(3),'m'); set(f,'LineWidth',2);
    f = plot3(Z_TPE(:,12)+mZ_TPE_uncalibrated(1),Z_TPE(:,13)+mZ_TPE_uncalibrated(2),Z_TPE(:,14)+mZ_TPE_uncalibrated(3),'m--'); set(f,'LineWidth',2);
    f = plot3(Z_MLPE(:,2),Z_MLPE(:,3),Z_MLPE(:,4),'g'); set(f,'LineWidth',2);
    f = plot3(Z_MLPE(:,22),Z_MLPE(:,23),Z_MLPE(:,24),'g--'); set(f,'LineWidth',2);
    f = plot3(Z_TPE(:,2),Z_TPE(:,3),Z_TPE(:,4),'b'); set(f,'LineWidth',2);
    f = plot3(Z_TPE(:,22),Z_TPE(:,23),Z_TPE(:,24),'b--'); set(f,'LineWidth',2);
    
    legend('VIVE', 'VIVE_{alignedToUncalibrated}','MLPE_{calibrated}','MLPE_{uncalibrated}','TPE_{calibrated}','TPE_{uncalibrated}')
end
%% YX
figure(6)
clf
hold on
if(~MLPEonly)
X_VIVE = [X_TPE(1:step:end,12)+mX_TPE(1);Y_TPE(1:step:end,12)+mY_TPE(1);Z_TPE(1:step:end,12)+mZ_TPE(1)];
Y_VIVE = [X_TPE(1:step:end,13)+mX_TPE(2);Y_TPE(1:step:end,13)+mY_TPE(2);Z_TPE(1:step:end,13)+mZ_TPE(2)];
Z_VIVE = [X_TPE(1:step:end,14)+mX_TPE(3);Y_TPE(1:step:end,14)+mY_TPE(3);Z_TPE(1:step:end,14)+mZ_TPE(3)];

X_VIVE_uncalibrated = [X_TPE(1:step:end,12)+mX_TPE_uncalibrated(1);Y_TPE(1:step:end,12)+mY_TPE_uncalibrated(1);Z_TPE(1:step:end,12)+mZ_TPE_uncalibrated(1)];
Y_VIVE_uncalibrated = [X_TPE(1:step:end,13)+mX_TPE_uncalibrated(2);Y_TPE(1:step:end,13)+mY_TPE_uncalibrated(2);Z_TPE(1:step:end,13)+mZ_TPE_uncalibrated(2)];
Z_VIVE_uncalibrated = [X_TPE(1:step:end,14)+mX_TPE_uncalibrated(3);Y_TPE(1:step:end,14)+mY_TPE_uncalibrated(3);Z_TPE(1:step:end,14)+mZ_TPE_uncalibrated(3)];

X_tpe = [X_TPE(1:step:end,2);Y_TPE(1:step:end,2);Z_TPE(1:step:end,2)];
Y_tpe = [X_TPE(1:step:end,3);Y_TPE(1:step:end,3);Z_TPE(1:step:end,3)];
Z_tpe = [X_TPE(1:step:end,4);Y_TPE(1:step:end,4);Z_TPE(1:step:end,4)];

X_tpe_uncalibrated = [X_TPE(1:step:end,22);Y_TPE(1:step:end,22);Z_TPE(1:step:end,22)];
Y_tpe_uncalibrated = [X_TPE(1:step:end,23);Y_TPE(1:step:end,23);Z_TPE(1:step:end,23)];
Z_tpe_uncalibrated = [X_TPE(1:step:end,24);Y_TPE(1:step:end,24);Z_TPE(1:step:end,24)];
else
X_VIVE = [X_MLPE(1:step:end,12)+mX_MLPE(1);Y_MLPE(1:step:end,12)+mY_MLPE(1);Z_MLPE(1:step:end,12)+mZ_MLPE(1)];
Y_VIVE = [X_MLPE(1:step:end,13)+mX_MLPE(2);Y_MLPE(1:step:end,13)+mY_MLPE(2);Z_MLPE(1:step:end,13)+mZ_MLPE(2)];
Z_VIVE = [X_MLPE(1:step:end,14)+mX_MLPE(3);Y_MLPE(1:step:end,14)+mY_MLPE(3);Z_MLPE(1:step:end,14)+mZ_MLPE(3)];

X_VIVE_uncalibrated = [X_MLPE(1:step:end,12)+mX_MLPE_uncalibrated(1);Y_MLPE(1:step:end,12)+mY_MLPE_uncalibrated(1);Z_MLPE(1:step:end,12)+mZ_MLPE_uncalibrated(1)];
Y_VIVE_uncalibrated = [X_MLPE(1:step:end,13)+mX_MLPE_uncalibrated(2);Y_MLPE(1:step:end,13)+mY_MLPE_uncalibrated(2);Z_MLPE(1:step:end,13)+mZ_MLPE_uncalibrated(2)];
Z_VIVE_uncalibrated = [X_MLPE(1:step:end,14)+mX_MLPE_uncalibrated(3);Y_MLPE(1:step:end,14)+mY_MLPE_uncalibrated(3);Z_MLPE(1:step:end,14)+mZ_MLPE_uncalibrated(3)];
end

X_mlpe = [X_MLPE(1:step:end,2);Y_MLPE(1:step:end,2);Z_MLPE(1:step:end,2)];
Y_mlpe = [X_MLPE(1:step:end,3);Y_MLPE(1:step:end,3);Z_MLPE(1:step:end,3)];
Z_mlpe = [X_MLPE(1:step:end,4);Y_MLPE(1:step:end,4);Z_MLPE(1:step:end,4)];

X_mlpe_uncalibrated = [X_MLPE(1:step:end,22);Y_MLPE(1:step:end,22);Z_MLPE(1:step:end,22)];
Y_mlpe_uncalibrated = [X_MLPE(1:step:end,23);Y_MLPE(1:step:end,23);Z_MLPE(1:step:end,23)];
Z_mlpe_uncalibrated = [X_MLPE(1:step:end,24);Y_MLPE(1:step:end,24);Z_MLPE(1:step:end,24)];

if(~MLPEonly)
    plot( X_VIVE, Y_VIVE, 'm')
    plot( X_VIVE_uncalibrated, Y_VIVE_uncalibrated, 'm--')
    plot( X_mlpe, Y_mlpe, 'g')
    plot( X_tpe, Y_tpe, 'b')
    plot( X_mlpe_uncalibrated, Y_mlpe_uncalibrated, 'g--')
    plot( X_tpe_uncalibrated, Y_tpe_uncalibrated, 'b--')
    legend('VIVE','VIVE','MLPE','TPE','MLPE','TPE')
else
    plot( X_VIVE, Y_VIVE, 'm')
    plot( X_VIVE_uncalibrated, Y_VIVE_uncalibrated, 'm--')
    plot( X_mlpe, Y_mlpe, 'g')
    plot( X_mlpe_uncalibrated, Y_mlpe_uncalibrated, 'g--')
    legend('VIVE','VIVE','MLPE','MLPE')
end
xlabel('x[m]')
ylabel('y[m]')
if(~MLPEonly)
axis([min([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_tpe;X_mlpe_uncalibrated;X_tpe_uncalibrated]),...
      max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_tpe;X_mlpe_uncalibrated;X_tpe_uncalibrated])+max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_tpe;X_mlpe_uncalibrated;X_tpe_uncalibrated])*0.4,...
      min([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_tpe;Y_mlpe_uncalibrated;Y_tpe_uncalibrated]),...
      max([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_tpe;Y_mlpe_uncalibrated;Y_tpe_uncalibrated])])
else
axis([min([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated]),...
      max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated])+max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated])*0.6,...
      min([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_mlpe_uncalibrated]),...
      max([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_mlpe_uncalibrated])])
end
matlab2tikz('directComparisonXY.tex','width','\fwidth','height','\fheight');
%% XZ
figure(7)
clf
hold on
if(~MLPEonly)
    plot( X_VIVE, Z_VIVE, 'm')
    plot( X_VIVE_uncalibrated, Z_VIVE_uncalibrated, 'm--')
    plot( X_mlpe, Z_mlpe, 'g')
    plot( X_tpe, Z_tpe, 'b')
    plot( X_mlpe_uncalibrated, Z_mlpe_uncalibrated, 'g--')
    plot( X_tpe_uncalibrated, Z_tpe_uncalibrated, 'b--')
    legend('VIVE','VIVE','MLPE','TPE','MLPE','TPE')
else
    plot( X_VIVE, Z_VIVE, 'm')
    plot( X_VIVE_uncalibrated, Z_VIVE_uncalibrated, 'm--')
    plot( X_mlpe, Z_mlpe, 'g')
    plot( X_mlpe_uncalibrated, Z_mlpe_uncalibrated, 'g--')
    legend('VIVE','VIVE','MLPE','MLPE')
end
xlabel('x[m]')
ylabel('z[m]')
if(~MLPEonly)
axis([min([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_tpe;X_mlpe_uncalibrated;X_tpe_uncalibrated]),...
      max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_tpe;X_mlpe_uncalibrated;X_tpe_uncalibrated])+max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated])*0.4,...
      min([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_tpe;Z_mlpe_uncalibrated;Z_tpe_uncalibrated]),...
      max([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_tpe;Z_mlpe_uncalibrated;Z_tpe_uncalibrated])])
else
    axis([min([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated]),...
      max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated])+max([X_VIVE;X_VIVE_uncalibrated;X_mlpe;X_mlpe_uncalibrated])*0.6,...
      min([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_mlpe_uncalibrated]),...
      max([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_mlpe_uncalibrated])])
end
matlab2tikz('directComparisonXZ.tex','width','\fwidth','height','\fheight');
%% ZY
figure(8)
clf
hold on
if(~MLPEonly)
    plot( Z_VIVE, Y_VIVE, 'm')
    plot( Z_VIVE_uncalibrated, Y_VIVE_uncalibrated, 'm--')
    plot( Z_mlpe, Y_mlpe, 'g')
    plot( Z_tpe, Y_tpe, 'b')
    plot( Z_mlpe_uncalibrated, Y_mlpe_uncalibrated, 'g--')
    plot( Z_tpe_uncalibrated, Y_tpe_uncalibrated, 'b--')
    legend('VIVE','VIVE','MLPE','TPE','MLPE','TPE')
else
    plot( Z_VIVE, Y_VIVE, 'm')
    plot( Z_VIVE_uncalibrated, Y_VIVE_uncalibrated, 'm--')
    plot( Z_mlpe, Y_mlpe, 'g')
    plot( Z_mlpe_uncalibrated, Y_mlpe_uncalibrated, 'g--')
    legend('VIVE','VIVE','MLPE','MLPE')
end
xlabel('z[m]')
ylabel('y[m]')
if(~MLPEonly)
axis([min([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_mlpe_uncalibrated;]),...
      max([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_mlpe_uncalibrated])+max([Z_VIVE;Z_VIVE_uncalibrated;Z_mlpe;Z_mlpe_uncalibrated])*0.6,...
      min([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_mlpe_uncalibrated]),...
      max([Y_VIVE;Y_VIVE_uncalibrated;Y_mlpe;Y_mlpe_uncalibrated])])
else
    
end
matlab2tikz('directComparisonZY.tex','width','\fwidth','height','\fheight');
%% load data
if(~MLPEonly)
    rotX_TPE = csvread('record_TPE_calibration_rotX.log',1);
    trotX_TPE = (rotX_TPE(:,1) - rotX_TPE(1,1))./1000000; % convert to milliseconds
    rotY_TPE = csvread('record_TPE_calibration_rotY.log',1);
    trotY_TPE = (rotY_TPE(:,1) - rotY_TPE(1,1))./1000000; % convert to milliseconds
    rotZ_TPE = csvread('record_TPE_calibration_rotZ.log',1);
    trotZ_TPE = (rotZ_TPE(:,1) - rotZ_TPE(1,1))./1000000; % convert to milliseconds
end

rotX_MLPE = csvread('record_MLPE_calibration_rotX.log',1);
trotX_MLPE = (rotX_MLPE(:,1) - rotX_MLPE(1,1))./1000000; % convert to milliseconds
rotY_MLPE = csvread('record_MLPE_calibration_rotY.log',1);
trotY_MLPE = (rotY_MLPE(:,1) - rotY_MLPE(1,1))./1000000; % convert to milliseconds
rotZ_MLPE = csvread('record_MLPE_calibration_rotZ.log',1);
trotZ_MLPE = (rotZ_MLPE(:,1) - rotZ_MLPE(1,1))./1000000; % convert to milliseconds
%%
if(~MLPEonly)
    figure(3)
    clf
    hold on
    % align the two coordinate frames using the first sample
    align = quatdivide([rotX_TPE(1,21) rotX_TPE(1,18:20)],[rotX_TPE(1,11) rotX_TPE(1,8:10)]);
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotX_TPE(:,11) rotX_TPE(:,8:10)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    [pitch_vive, roll_vive, yaw_vive] = quat2angle([rotX_TPE(:,21) rotX_TPE(:,18:20)], 'XYZ');
    pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;
    
    rmserotX_TPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotX_TPE(1:step:end),pitch_vive(1:step:end),'r')
    plot(trotX_TPE(1:step:end),roll_vive(1:step:end),'g')
    plot(trotX_TPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

    plot(trotX_TPE(1:step:end),pitch(1:step:end),'r--')
    plot(trotX_TPE(1:step:end),roll(1:step:end),'g--')
    plot(trotX_TPE(1:step:end),abs(yaw(1:step:end)),'b--')
    
    % align the two coordinate frames using the first sample
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotX_TPE(:,31) rotX_TPE(:,28:30)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    rmserotX_TPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotX_TPE(1:step:end),pitch(1:step:end),'r-.')
    plot(trotX_TPE(1:step:end),roll(1:step:end),'g-.')
    plot(trotX_TPE(1:step:end),abs(yaw(1:step:end)),'b-.')
    xlabel('t[ms]')
    ylabel('euler[degree]')
    axis([0,trotX_MLPE(end)+trotX_MLPE(end)*0.15,...
        min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
        max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
    legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
    matlab2tikz('rotX_TPEComparison.tex','width','\fwidth','height','\fheight');
end
%%
if(~MLPEonly)
    figure(4)
    clf
    hold on
    % align the two coordinate frames using the first sample
    align = quatdivide([rotY_TPE(1,21) rotY_TPE(1,18:20)],[rotY_TPE(1,11) rotY_TPE(1,8:10)]);
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotY_TPE(:,11) rotY_TPE(:,8:10)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    [pitch_vive, roll_vive, yaw_vive] = quat2angle([rotY_TPE(:,21) rotY_TPE(:,18:20)], 'XYZ');
    pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;
    
    rmserotY_TPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotY_TPE(1:step:end),pitch_vive(1:step:end),'r')
    plot(trotY_TPE(1:step:end),roll_vive(1:step:end),'g')
    plot(trotY_TPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

    plot(trotY_TPE(1:step:end),pitch(1:step:end),'r--')
    plot(trotY_TPE(1:step:end),roll(1:step:end),'g--')
    plot(trotY_TPE(1:step:end),abs(yaw(1:step:end)),'b--')
    
    % align the two coordinate frames using the first sample
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotY_TPE(:,31) rotY_TPE(:,28:30)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    rmserotY_TPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotY_TPE(1:step:end),pitch(1:step:end),'r-.')
    plot(trotY_TPE(1:step:end),roll(1:step:end),'g-.')
    plot(trotY_TPE(1:step:end),abs(yaw(1:step:end)),'b-.')
    xlabel('t[ms]')
    ylabel('euler[degree]')
    axis([0,trotY_MLPE(end)+trotY_MLPE(end)*0.155,...
        min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
        max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
    legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
    matlab2tikz('rotY_TPEComparison.tex','width','\fwidth','height','\fheight');
end
%%
if(~MLPEonly)
    figure(5)
    clf
    hold on
    % align the two coordinate frames using the first sample
    align = quatdivide([rotZ_TPE(1,21) rotZ_TPE(1,18:20)],[rotZ_TPE(1,11) rotZ_TPE(1,8:10)]);
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotZ_TPE(:,11) rotZ_TPE(:,8:10)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    [pitch_vive, roll_vive, yaw_vive] = quat2angle([rotZ_TPE(:,21) rotZ_TPE(:,18:20)], 'XYZ');
    pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;
    
    rmserotZ_TPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotZ_TPE(1:step:end),pitch_vive(1:step:end),'r')
    plot(trotZ_TPE(1:step:end),roll_vive(1:step:end),'g')
    plot(trotZ_TPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

    plot(trotZ_TPE(1:step:end),pitch(1:step:end),'r--')
    plot(trotZ_TPE(1:step:end),roll(1:step:end),'g--')
    plot(trotZ_TPE(1:step:end),abs(yaw(1:step:end)),'b--')
    
    % align the two coordinate frames using the first sample
    [pitch, roll, yaw] = ...
        quat2angle(quatmultiply([rotZ_TPE(:,31) rotZ_TPE(:,28:30)],align), 'XYZ');
    pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;
    
    rmserotZ_TPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))
    
    plot(trotZ_TPE(1:step:end),pitch(1:step:end),'r-.')
    plot(trotZ_TPE(1:step:end),roll(1:step:end),'g-.')
    plot(trotZ_TPE(1:step:end),abs(yaw(1:step:end)),'b-.')
    xlabel('t[ms]')
    ylabel('euler[degree]')
    axis([0,trotZ_MLPE(end)+trotZ_MLPE(end)*0.155,...
        min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
        max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
    legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
    matlab2tikz('rotZ_TPEComparison.tex','width','\fwidth','height','\fheight');
end
%%
figure(9)
clf
hold on
% align the two coordinate frames using the first sample
align = quatdivide([rotX_MLPE(1,21) rotX_MLPE(1,18:20)],[rotX_MLPE(1,11) rotX_MLPE(1,8:10)]);
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotX_MLPE(:,11) rotX_MLPE(:,8:10)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

[pitch_vive, roll_vive, yaw_vive] = quat2angle([rotX_MLPE(:,21) rotX_MLPE(:,18:20)], 'XYZ');
pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;

rmserotX_MLPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotX_MLPE(1:step:end),pitch_vive(1:step:end),'r')
plot(trotX_MLPE(1:step:end),roll_vive(1:step:end),'g')
plot(trotX_MLPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

plot(trotX_MLPE(1:step:end),pitch(1:step:end),'r--')
plot(trotX_MLPE(1:step:end),roll(1:step:end),'g--')
plot(trotX_MLPE(1:step:end),abs(yaw(1:step:end)),'b--')

% align the two coordinate frames using the first sample
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotX_MLPE(:,31) rotX_MLPE(:,28:30)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

rmserotX_MLPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotX_MLPE(1:step:end),pitch(1:step:end),'r-.')
plot(trotX_MLPE(1:step:end),roll(1:step:end),'g-.')
plot(trotX_MLPE(1:step:end),abs(yaw(1:step:end)),'b-.')
xlabel('t[ms]')
ylabel('euler[degree]')
axis([0,trotX_MLPE(end)+trotX_MLPE(end)*0.155,...
    min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
    max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
matlab2tikz('rotX_MLPEComparison.tex','width','\fwidth','height','\fheight');
%%
figure(10)
clf
hold on
% align the two coordinate frames using the first sample
align = quatdivide([rotY_MLPE(1,21) rotY_MLPE(1,18:20)],[rotY_MLPE(1,11) rotY_MLPE(1,8:10)]);
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotY_MLPE(:,11) rotY_MLPE(:,8:10)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

[pitch_vive, roll_vive, yaw_vive] = quat2angle([rotY_MLPE(:,21) rotY_MLPE(:,18:20)], 'XYZ');
pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;

rmserotY_MLPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotY_MLPE(1:step:end),pitch_vive(1:step:end),'r')
plot(trotY_MLPE(1:step:end),roll_vive(1:step:end),'g')
plot(trotY_MLPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

plot(trotY_MLPE(1:step:end),pitch(1:step:end),'r--')
plot(trotY_MLPE(1:step:end),roll(1:step:end),'g--')
plot(trotY_MLPE(1:step:end),abs(yaw(1:step:end)),'b--')

% align the two coordinate frames using the first sample
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotY_MLPE(:,31) rotY_MLPE(:,28:30)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

rmserotY_MLPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotY_MLPE(1:step:end),pitch(1:step:end),'r-.')
plot(trotY_MLPE(1:step:end),roll(1:step:end),'g-.')
plot(trotY_MLPE(1:step:end),abs(yaw(1:step:end)),'b-.')
xlabel('t[ms]')
ylabel('euler[degree]')
axis([0,trotY_MLPE(end)+trotY_MLPE(end)*0.155,...
    min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
    max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
matlab2tikz('rotY_MLPEComparison.tex','width','\fwidth','height','\fheight');
%%
figure(11)
clf
hold on
% align the two coordinate frames using the first sample
align = quatdivide([rotZ_MLPE(1,21) rotZ_MLPE(1,18:20)],[rotZ_MLPE(1,11) rotZ_MLPE(1,8:10)]);
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotZ_MLPE(:,11) rotZ_MLPE(:,8:10)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

[pitch_vive, roll_vive, yaw_vive] = quat2angle([rotZ_MLPE(:,21) rotZ_MLPE(:,18:20)], 'XYZ');
pitch_vive = pitch_vive *180/pi; roll_vive = roll_vive *180/pi; yaw_vive = yaw_vive *180/pi;

rmserotZ_MLPE_calibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotZ_MLPE(1:step:end),pitch_vive(1:step:end),'r')
plot(trotZ_MLPE(1:step:end),roll_vive(1:step:end),'g')
plot(trotZ_MLPE(1:step:end),abs(yaw_vive(1:step:end)),'b')

plot(trotZ_MLPE(1:step:end),pitch(1:step:end),'r--')
plot(trotZ_MLPE(1:step:end),roll(1:step:end),'g--')
plot(trotZ_MLPE(1:step:end),abs(yaw(1:step:end)),'b--')

% align the two coordinate frames using the first sample
[pitch, roll, yaw] = ...
    quat2angle(quatmultiply([rotZ_MLPE(:,31) rotZ_MLPE(:,28:30)],align), 'XYZ');
pitch = pitch *180/pi; roll = roll *180/pi; yaw = yaw *180/pi;

rmserotZ_MLPE_uncalibrated = sqrt(sum(([pitch roll abs(yaw)] - [pitch_vive roll_vive abs(yaw_vive)]).^2/length(pitch)))

plot(trotZ_MLPE(1:step:end),pitch(1:step:end),'r-.')
plot(trotZ_MLPE(1:step:end),roll(1:step:end),'g-.')
plot(trotZ_MLPE(1:step:end),abs(yaw(1:step:end)),'b-.')
xlabel('t[ms]')
ylabel('euler[degree]')
axis([0,trotZ_MLPE(end)+trotZ_MLPE(end)*0.155,...
    min(min([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)])),...
    max(max([pitch(1:step:end) roll(1:step:end) yaw(1:step:end) pitch_vive(1:step:end) roll_vive(1:step:end) yaw_vive(1:step:end)]))])
legend('pitch','roll','yaw','pitch','roll','yaw','pitch','roll','yaw')
matlab2tikz('rotZ_MLPEComparison.tex','width','\fwidth','height','\fheight');