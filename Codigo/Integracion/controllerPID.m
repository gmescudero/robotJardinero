function [v,w,wpReached] = controllerPID(...
    controller,...  % Controller parameters
    Xk,...          % Current position
    wp)             % Target waypoint

persistent e_ se

% Angle to waypoint
angwp = atan2(wp(2)-Xk(2),wp(1)-Xk(1));

% Error values
e = (sin(angwp-Xk(3)) + (1-cos(angwp-Xk(3))));
if isempty(e_) || isempty(se)
    e_ = e;
    se = e;
else
    se = se + e;
end

% PID controller for angle to waypoint
Kp = controller.Kp;
Ki = controller.Ki;
Kd = controller.Kd;
h  = controller.sampleT;
w = Kp*e + Ki*h*se + Kd*(e-e_)/h; 
e_ = e; 

% Linear vel adjust
if abs(angwp-Xk(3)) < pi/10
    v = 0.2;
else
    v = 0.075;
end

% Comprueba si esta en el wp y si es asi pasa al siguiente wp
wpReached = false;
if sqrt((wp(2)-Xk(2))^2+(wp(1)-Xk(1))^2)<0.1
    wpReached = true;
end
end

