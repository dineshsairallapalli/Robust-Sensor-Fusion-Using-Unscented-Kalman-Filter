function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %uCurr and covar_curr are the updated mean and covariance respectively

% Define measurement matrix
I = [eye(3),zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),eye(3),zeros(3),zeros(3),zeros(3)];
% Predicted measurement using estimated state
z = I * uEst + zeros(6,1); %(assuming H is identity matrix)
% Measurement sensitivity matrix
Ct = I ;      
% Measurement noise covariance
R = eye(6,6) * 0.2;
% Kalman gain calculation
Kt = covarEst * Ct' /(Ct * covarEst* Ct' + R );
% Update mean state estimate
uCurr = uEst + Kt * (z_t - z );
% Update covariance estimate
covar_curr = covarEst- Kt* Ct * covarEst;

end

