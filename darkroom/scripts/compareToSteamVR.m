close all
clear 
g.figure1=figure('Name','simulated calibration values');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], ...
          'XGrid','on','YGrid','on','ZGrid','on', ...
          'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]);   
view(0,0)
A = csvread('record_calibration_03032018_203630.log',1);
hold on
plot3(A(:,2),A(:,3),A(:,4))
plot3(A(:,9),A(:,10),A(:,11),'r')