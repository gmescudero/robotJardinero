pos=apoloGetOdometry('Marvin')
A=[pos]
for i = 1:100*100
apoloMoveMRobot('Marvin', [0.05, 0.05], 0.1);
apoloUpdate()
a=apoloGetOdometry('Marvin');
testLaserData('LMS100','Marvin',1);
A = [A ; a];
end
plot(A(:,1), A(:,2),'b')


% a=testLaserData('LMS100','Marvin',100)
apoloPlaceMRobot('Marvin',[0 0 0],0)
apoloResetOdometry('Marvin')
apoloUpdate