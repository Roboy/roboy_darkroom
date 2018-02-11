function plotLighthouseSensors(rel_pos, pose, plottype, fontsize)
%plotLighthouseSensors 
rel_pos = [rel_pos';ones(size(rel_pos,1),1)'];
pos = pose*rel_pos;
hold on
for i=1:size(rel_pos,2)
plot3(pos(1,i),pos(2,i),pos(3,i),plottype);
text(pos(1,i),pos(2,i),pos(3,i),num2str(i),'FontSize',fontsize);
end
end

