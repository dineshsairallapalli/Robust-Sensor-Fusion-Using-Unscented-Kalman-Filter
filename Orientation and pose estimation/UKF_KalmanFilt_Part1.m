clear; % Clear variables
addpath('../data')
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM

% Initialize dataset
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);

% Get Vicon measurements
Z = sampledVicon(1:6,:);

% Set initial state estimate
uPrev = vertcat(sampledVicon(1:9,1), zeros(6,1)); % Copy the Vicon initial state
covarPrev = 0.1 * eye(15); % Initial covariance

% Initialize array to save estimated states
savedStates = zeros(15, length(sampledTime));

% Initialize previous time variable
prevTime = 0;

% Loop through each time step
for i = 1:length(sampledTime)
    % Get angular velocity and acceleration at current time step
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    
    % Calculate time difference from previous time step
    currTime = sampledTime(i);
    dt = sampledTime(i) - prevTime;
    prevTime = currTime;
    
    % Get Vicon measurement at current time step
    z_t = Z(:,i);
    
    % Prediction step: Estimate the next state
    [covarEst, uEst] = pred_step(uPrev, covarPrev, angVel, acc, dt);
    
    % Update step: Incorporate measurement to refine state estimate
    [uCurr, covarcurr] = upd_step(z_t, covarEst, uEst);
    
    % Update previous state and covariance for the next iteration
    uPrev = uCurr;
    covarPrev = covarcurr;
    
    % Save the current state estimate
    savedStates(:,i) = uCurr;
end

% Plot the estimated states
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);
