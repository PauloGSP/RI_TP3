
% Manually Calibrate the camera
cameraCalibrator
pause()

% Show the calibration results
showReprojectionErrors(cameraParams);

% Save the camera parameters to a file
save('cameraParams.mat', 'cameraParams');
save('estimationErrors.mat', 'estimationErrors');
