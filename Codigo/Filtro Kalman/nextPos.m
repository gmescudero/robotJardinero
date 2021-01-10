function [X_kp1] = nextPos(...
    X_k, ...    % [] Previous position
    U_kp1)      % [] Odometry delta

%% compute next position
X_kp1 = X_k + U_kp1;

end

