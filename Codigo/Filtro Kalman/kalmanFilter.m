function [Xest_kp1_kp1,P_kp1_kp1] = kalmanFilter(...
    X_k, ...    % Last known position
    P_k, ...    % Last covariance matrix
    U_kp1, ... 	% Odometry delta
    Z_kp1)      % Sensor Readings

% TODO build this matrixes correctly
Q_k = eye(size(X_k(1)));
H_kp1 = eye(size(X_k(1)));
R_kp1 = eye(size(X_k(1)));

%% Prediction
% New estimated position from Odometry
Xest_kp1_k = nextPos(X_k,U_kp1,0); 
F_Xest_k    = 1; % nextPos derivated over Xest_k
F_U_kp1     = 1; % nextPos derivated over U_kp1
F_w_k       = 1; % nextPos derivated over w_k

% Variances and Covariances
P_kp1_k = F_Xest_k*P_k*F_Xest_k' + F_w_k*Q_k*F_w_k'; 

% Estimated masurements
Zest_kp1 = pos2measures(Xest_kp1_k,0); 

%% Observation
% Z_kp1 = Z_k

%% Comparation
Yps_kp1 = Z_kp1 - Zest_kp1;
S_kp1 = H_kp1 * P_kp1_k * H_kp1' + R_kp1;

%% Correction
W_kp1 = P_kp1_k * H_kp1'/(S_kp1);

Xest_kp1_kp1 = Xest_kp1_k + W_kp1 * Yps_kp1;
P_kp1_kp1 = P_kp1_k - (W_kp1 / S_kp1) * W_kp1;

end

