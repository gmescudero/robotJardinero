function [vf,vg] = trajectoryControlP(way,X,Kf,Kg,vfMax,vgMax)

e = [way(1)-X(1), way(2)-X(2), atan2(way(2)-X(2),way(1)-X(1))-X(3)];
if abs(e(1))>0.1 && abs(e(2))>0.1
    vg = max( -vgMax, min( vgMax, Kg*e(3) ) );
    if abs(vg) < 0.1
        vf = max( -vfMax, min( vfMax, Kf*sqrt(e(1)^2 + e(2)^2) ) );
    else
        vf = 0;
    end
else % reached
    vf = 0;
    vg = 0;
end
end

