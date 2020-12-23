function [Zest] = pos2measures(...
    X, ...  % [] Position
    v)      % []/0 Measuring noise
% Land Marks positions
LM = [... % LM(lmID,[X,Y])
    [10.35  5.55];
    [0      0   ];
    [0      13.5];
    [26.5	13.5];
    [26.5	0   ];
    [9.2	6.75];
    [11.3	6.75];
    [10.4	8   ];
    [15.7	9.9 ];
    [15.5	3.3 ];
    [20.5	7.3 ];
    [20.5	6.1	];
    [20.3	12.2];
    [20.3	1.1 ];
    [24     5.6 ];
    [23.6	7.6 ];
    [22.3	8.4 ];
    [22.5	5.5 ];
    [11.3	0.7 ];
    [11.4	12.4];
    [5.8	10.1];
    [5.9	4.0 ];
    [1.4	7.2 ];
];
[lmNum,~] = size(LM);
Zest = zeros(lmNum,2);

%% Compute
for i=1:lmNum
    % distance 
    Zest(i,1) = sqrt((LM(i,1) - X(1))^2 + (LM(i,2) - X(2))^2);
    % angle
    Zest(i,2) = atan2(LM(i,2)-X(2),LM(i,1) - X(1)) - X(3);
end

% Add the noise
Zest = Zest + v;

end

