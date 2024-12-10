function [uCurr, covar_curr] = upd_step(z_t, covarEst, uEst, omega)
% Update step of the Kalman filter

% Parameter Definition:
% z_t - sensor data at the time step
% covarEst - estimated covariance of the state
% uEst - estimated mean of the state
% omega - angular velocity

% Initialize parameters
alpha = 0.001; % Used to calculate weighted matrix
k = 1; % Determines Sigma Points spread
beta = 2; % Used to calculate weighted matrix
n = 15; % Number of sigma points
vt = zeros(3, 1); % Zero velocity
covaraug = chol(covarEst, 'lower'); % Cholesky decomposition of the covariance matrix
lambdaprime = ((alpha)^2 * (n + k) - n); % Calculate lambda prime

% Define camera frame offset
r_cb_b = [-0.04 * cos(0.785), 0, -0.03]; % Camera offset
skew_r_cb_b = [0, 0.03, 0.0283; -0.03, 0, -0.0283; -0.0283, 0.0283, 0]; % Skew symmetric matrix

% Initialize matrices for positive and negative sigma points
Xestplus = [];
Xestminus = [];

roll = uEst(4, 1); %x orientation
pitch = uEst(5, 1); %y orientation
yaw = uEst(6, 1); %z orientation 

Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)]; %Rotation around X-axis
Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)]; %Rotation aroud Y-axis
Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1]; %Rotation around Z-axis

% Rotation of the body frame wrt camera
Rbc = rotz(-45) * rotx(180); % Rotation from body frame to camera frame
Rcb = Rbc'; % Transpose of Rbc

Rbw = (Rz * Ry * Rx); % Rotation from body frame to world frame
Rwb = Rbw'; % Transpose of Rbw

g0 = Rbc * Rwb * uEst(7:9) - Rbc * skew_r_cb_b * Rcb * omega' + vt; % Initial g value

% Generate sigma points
for i = 1:n
    Xestp = uEst + sqrt(n + lambdaprime) * covaraug(:, i); % Positive sigma points
    Xestm = uEst - sqrt(n + lambdaprime) * covaraug(:, i); % Negative sigma points
    Xestplus = [Xestplus, Xestp];
    Xestminus = [Xestminus, Xestm];
end    
Xest = [uEst, Xestplus, Xestminus]; % Combined sigma points

% Initialize matrices for storing g values
gplus = [];
gminus = [];

% Calculate g values for each sigma point
for i = 1:n
    % Extract Euler angles
    rollplus = Xestplus(4, i);
    pitchplus = Xestplus(5, i);
    yawplus = Xestplus(6, i);
    
    rollminus = Xestminus(4, i);
    pitchminus = Xestminus(5, i);
    yawminus = Xestminus(6, i);

    % Calculate rotation matrices for positive sigma points
    Rxplus = [1, 0, 0; 0, cos(rollplus), -sin(rollplus); 0, sin(rollplus), cos(rollplus)];
    Ryplus = [cos(pitchplus), 0, sin(pitchplus); 0, 1, 0; -sin(pitchplus), 0, cos(pitchplus)];
    Rzplus = [cos(yawplus), -sin(yawplus), 0; sin(yawplus), cos(yawplus), 0; 0, 0, 1];
    Rbwplus = (Rzplus * Ryplus * Rxplus); % Rotation from body frame to world frame
    Rwbplus = Rbwplus'; % Transpose of Rbwplus

    % Calculate g values for positive sigma points
    gp = Rbc * Rwbplus * Xestplus(7:9, i) - Rbc * skew_r_cb_b * Rcb * omega' + vt;
    gplus = [gplus, gp];
    
    % Calculate rotation matrices for negative sigma points
    Rxminus = [1, 0, 0; 0, cos(rollminus), -sin(rollminus); 0, sin(rollminus), cos(rollminus)];
    Ryminus = [cos(pitchminus), 0, sin(pitchminus); 0, 1, 0; -sin(pitchminus), 0, cos(pitchminus)];
    Rzminus = [cos(yawminus),-sin(yawminus), 0; sin(yawminus), cos(yawminus), 0; 0, 0, 1];
    Rbwminus = (Rzminus * Ryminus * Rxminus); % Rotation from body frame to world frame
    Rwbminus = Rbwminus'; % Transpose of Rbwminus

    % Calculate g values for negative sigma points
    gm = Rbc * Rwbminus * Xestminus(7:9, i) - Rbc * skew_r_cb_b * Rcb * omega' + vt;
    gminus = [gminus, gm];
end

g = [g0, gplus, gminus]; % Combine g values

% Calculate weights
Wc0 = (lambdaprime / (n + lambdaprime)) + (1 - (alpha)^2 + beta);
Wci = 1 / (2 * (n + lambdaprime));
Wu0 = lambdaprime / (n + lambdaprime);
Wui = Wci;

% Calculate mean estimate
Ut0 = Wu0 * g(:, 1);
Uti = 0;
for i = 2:2 * n + 1
    Uti = Uti + Wui * g(:, i);
end
Ut = Ut0 + Uti;

% Calculate covariance estimate
S0 = Wc0 * (g(:, 1) - Ut) * (g(:, 1) - Ut)';
Si = 0;
for i = 2:2 * n + 1
    Si = Si + Wci * (g(:, i) - Ut) * (g(:, i) - Ut)';
end
S = S0 + Si + 0.5 * eye(3); % Add measurement noise covariance
C0 = Wc0 * (Xest(:, 1) - uEst) * (g(:, 1) - Ut)';
Ci = 0;
for i = 2:2 * n + 1
    Ci = Ci + Wci * (Xest(:, i) - uEst) * (g(:, i) - Ut)';
end
C = C0 + Ci;

% Kalman gain
Kt = C / S;

% Update mean and covariance
uCurr = uEst + Kt * (z_t' - Ut);
covar_curr = covarEst - Kt * S * Kt';

end
