function [covarEst, uEst] = pred_step(uPrev, covarPrev, angVel, acc, dt)
% Prediction step of the Kalman filter

% Parameters:
% uPrev - mean of the previous state
% covarPrev - covariance of the previous state
% angVel - angular velocity input at the time step
% acc - acceleration at the time step
% dt - time step duration

% Initialize parameters
alpha = 0.001; %Used to calculate weighted matrix
k = 1; %Determines Sigma Points spread
beta = 2; %Used to calculate weighted matrix
n = 27;  %No of Samples

% Process noise covariance matrix
Q = eye(12)*0.001; %Co-Variance of Noise

% Augmented covariance matrix
covaraug1 = [covarPrev, zeros(15, 12); zeros(12, 15), Q]; %Augumented Matrix of Co-variance
covaraug = chol(covaraug1, 'lower'); % Cholesky decomposition

% Augmented mean vector
uaug = [uPrev; zeros(12, 1)]; %MAugumented Mean Matrix

% Calculate lambda prime
lambdaprime = ((alpha)^2 * (n + k) - n); %lambdaprime

% Generate sigma points
X_iaugplus = []; %Initiliazing with Null value
X_iaugminus = []; %Initiliazing with Null value

for i = 1:length(covaraug) %loop runs tull the length of the augumented matrix of co-variance
    X_augp = uaug + sqrt(n + lambdaprime) * covaraug(:, i); % Compute Sigma Points with the augmented
    X_augm = uaug - sqrt(n + lambdaprime) * covaraug(:, i); %Compute Sigma Points with the augmented
    X_iaugplus = [X_iaugplus, X_augp];
    X_iaugminus = [X_iaugminus, X_augm];
end
X0aug = uaug; %Previous mean augumented matrix
Xaug = [X0aug, X_iaugplus, X_iaugminus]; %Xaugumented Matrix
 
% Initialize variables for state prediction
xdotaug = [];
xt = [];
g = [0; 0; -9.8]; % Gravity vector

% Calculate weights
Wc0 = (lambdaprime / (n + lambdaprime)) + (1 - alpha^2 + beta);
Wci = 1 / (2 * (n + lambdaprime));
Wu0 = lambdaprime / (n + lambdaprime);
Wui = Wci;

% Perform prediction for each sigma point
for j = 1:2 * n + 1 %runs till 27 
    % Extract variables from sigma points
    x = Xaug(1, j);  %position
    y = Xaug(2, j);  %position
    z = Xaug(3, j);  %position
    roll = Xaug(4, j); %orientation
    pitch = Xaug(5, j); %orientation
    yaw = Xaug(6, j); %orientation
    x3 = Xaug(7:9, j); %linear velocity
    x4 = Xaug(10:12, j); %gyroscope bias
    x5 = Xaug(13:15, j); %accelerometer bias
    ng = Xaug(16:18, j); %noise in gyroscope
    na = Xaug(19:21, j); %noise in accelerometer
    nbg = Xaug(22:24, j); %Drift in the gyroscope bias is described by a Gaussian, white noise process
    nba = Xaug(25:27, j); %%Drift in the accelerometer bias is described by a Gaussian, white noise process

    
    G = [(cos(yaw) * sin(pitch)) / cos(pitch), (sin(pitch) * sin(yaw)) / cos(pitch), 1;
         -sin(yaw), cos(yaw), 0;
         cos(yaw) / cos(pitch), sin(yaw) / cos(pitch), 0];
    Ginv = flip(G);
    
    % Compute rotation matrices
    R=eul2rotm([yaw,pitch,roll],'ZYX'); %Euler angle rotation matrix along ZYX

    % Compute state derivatives with noise
    xdot = [x3; Ginv * R * (angVel - x4 - ng); g + (R) * (acc - x5 - na); nbg; nba];
    xdotaug = [xdotaug, xdot];

    % Euler integration to predict state
    xt(1:9, j) = xdotaug(1:9, j) * dt + Xaug(1:9, j);
    xt(10:15, j) = xdotaug(10:15, j) + Xaug(10:15, j);
end

% Calculate mean state estimate
ut0 = Wu0 * xt(:, 1);
uti = 0;
for i = 2:2*n + 1
    uti = uti+Wui*xt(:, i);
end
uEst = ut0 + uti;

% Calculate covariance estimate
covar0 = (Wc0 * (xt(:, 1) - uEst) * (xt(:, 1) - uEst)');
covari = 0;
for i = 2:2 * n + 1
    covari = (covari + Wci * (xt(:, i) - uEst) * (xt(:, i) - uEst)');
end
covarEst = covar0 + covari;

end
