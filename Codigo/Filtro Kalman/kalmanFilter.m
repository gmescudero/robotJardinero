function [Xest_kp1_kp1,P_kp1_kp1] = kalmanFilter(...
    X_k, ...    % Last known position
    P_k, ...    % Last covariance matrix
    U_kp1, ... 	% Odometry delta
    Z_kp1)      % Sensor Readings

% TODO build this matrixes correctly
Q_k = eye(size(X_k(1)));
R_kp1 = eye(size(X_k(1)));

%% Prediction
% New estimated position from Odometry
Xest_kp1_k = nextPos(X_k,U_kp1); 
F_Xest_k   = 1; % nextPos derivated over Xest_k

% Variances and Covariances
P_kp1_k = F_Xest_k*P_k*F_Xest_k' + Q_k'; 

% Estimated masurements and H matrix
[Zest_kp1,H_kp1] = pos2measures(Xest_kp1_k); 
for i = 1:length(Zest_kp1(:,1))/3
    ind = (i-1)*3; % index
    if 0 == Z_kp1(ind+1)
        Z_kp1(ind+1) = Zest_kp1(ind+1);
        Z_kp1(ind+2) = Zest_kp1(ind+2);
        Z_kp1(ind+3) = Zest_kp1(ind+3);
    end
end
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

