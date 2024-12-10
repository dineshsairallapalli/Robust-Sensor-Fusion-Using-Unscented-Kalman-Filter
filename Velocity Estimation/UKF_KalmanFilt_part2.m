clear; % Clear variables
addpath('../data')

datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM

% Initialize data
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);

% Set initial condition
uPrev = vertcat(sampledVicon(1:9, 1), zeros(6, 1)); % Copy the Vicon Initial state
covarPrev = 0.01 * eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); % Initialize array for saving state history
prevTime = 0; % Initialize previous time

vel = proj2Data.linearVel; % Linear velocity from project data
angVel2 = proj2Data.angVel; % Angular velocity from project data

%% Calculate Kalman Filter
for i = 1:length(sampledTime)
    %% Loop through each time step
    angVel = sampledData(i).omg; % Angular velocity from sampled data
    acc = sampledData(i).acc; % Acceleration from sampled data
    currTime = proj2Data.time(i); % Current time from project data
    dt = sampledTime(i) - prevTime; % Time difference

    % Perform prediction step of Kalman filter
    [covarEst, uEst] = pred_step(uPrev, covarPrev, angVel, acc, dt);

    % Perform update step of Kalman filter
    [uCurr, covarcurr] = upd_step(vel(i, :), covarEst, uEst, angVel2(i, :));

    savedStates(:, i) = uCurr(:); % Save current state
    covarPrev = covarcurr; % Update previous covariance
    uPrev = uCurr; % Update previous state
    prevTime = currTime; % Update previous time
end

% Plot the saved states
plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);
