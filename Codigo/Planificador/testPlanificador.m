
load jardinBinMap2.mat 
goal = [26, 6];
xSize = 26.5;
cellsPerMeter = round(length(BW(1,:))/xSize);
Xk = [1,1,0];

[ret,wp] = mappingAndPlan(2,BW,Xk,goal,cellsPerMeter);
if 0 == ret
    disp ('failed');
else
    imshow (~BW)
    hold on
    for i = 1:length(wp(:,1))
        x = wp(i,1)*cellsPerMeter;
        y = wp(i,2)*cellsPerMeter;
        plot(x,y, 'o');
    end
end