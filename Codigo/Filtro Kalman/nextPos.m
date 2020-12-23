function [X_kp1] = nextPos(...
    X_k, ...    % [] Previous position
    U_kp1, ...  % [] Odometry delta
    w_k)        % []/0 Process noise

%% compute next position
X_kp1 = X_k + U_kp1 + w_k;

end

