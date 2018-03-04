close all
clear 
g.figure1=figure('Name','compare steamVR and custom tracking');  
%config 3d plot
g.axes=axes('Linewidth',2,'DataAspectRatio',[1 1 1], 'XGrid','on','YGrid','on','ZGrid','on');   %, ...
          %, ...
          %'XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]
view(0,0)
A = csvread('record_calibration_rotY.log',1);
%%
m = mean(A(:,2:4)-A(:,12:14))
hold on
xlabel('x')
ylabel('y')
zlabel('z')
plot3(A(:,2),A(:,3),A(:,4))
plot3(A(:,12)+m(1),A(:,13)+m(2),A(:,14)+m(3),'r')

%%
h.figure1=figure('Name','distance');  
plot(A(:,1), sqrt((A(:,2)-A(:,12)).^2+(A(:,3)-A(:,13)).^2+(A(:,4)-A(:,14)).^2))
%%
h.figure1=figure('Name','velocity');  
hold on
plot(A(:,1), A(:,5),'r')
plot(A(:,1), A(:,6),'g')
plot(A(:,1), A(:,7),'b')
%%
% h.figure1=figure('Name','error');  
% hold on
% plot(A(:,1), sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2),'r')
% [C1,lag1] = xcorr(sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2),A(:,6));
% plot(A(:,1),C1(1:2:end,1)/500+mean(sqrt((A(:,2)-A(:,12)+m(1)).^2+(A(:,3)-A(:,13)+m(2)).^2+(A(:,4)-A(:,14)+m(3)).^2)),'g')
