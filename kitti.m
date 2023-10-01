clc
close all
clear all

M = 45; % dataset max index
% Construct file names to read the KITTI LiDAR data files jumping 5
fname = arrayfun(@(idx) sprintf('./kitti/%d.pcd', idx), 0:5:M, 'uniformoutput', false);

fprintf('KITTI LiDAR data loading from specified directory.\n');

% Load KITTI LiDAR data (assuming PCD format)
V = cellfun(@(fname) pcread(fname), fname, 'uniformoutput', false);

df = 1; % downsample degree
[V, ~] = cellfun(@(V) deal(V.Location(1:df:end,:)'), V, 'uniformoutput', false);

% Initialize parameters for the jrmpc function
maxNumIter = 100; % Maximum number of iterations
epsilon = 1e-6;   % Artificial covariance flatten
updatePriors = 0; % Do not update priors during iterations

Ri = eye(3);  % Initial guess for rotation matrix
ti = zeros(3, 1);  % Initial guess for translation vector
Xin = V{1}(:,unique(round(linspace(1,size(V{1},2),450))));
accum_matrix = eye(4);

% Specify the output file
outputFilePath = './res/poses07.txt';
fid = fopen(outputFilePath, 'w');

fprintf('Registration Start\n');
for j = 2:numel(V)
    fprintf('Registration on %d and %d:\n', j-1, j);
    v1 = V{j-1};
    v2 = V{j};

    % Call jrmpc on two consecutive frames
    [R, t, Xout] = jrmpc({v1, v2}, Xin, 'R',{Ri, Ri}, 't', {ti, ti}, 'maxNumIter', maxNumIter, 'epsilon', epsilon, 'updatePriors', updatePriors);

    % Update the guesses
    Xin = Xout;
    Ri = R{1};
    ti = t{1};

    % Solve the transformation matrix
    R1to2 = inv(R{2}) * R{1};
    t1to2 = inv(R{2}) * (t{1}-t{2});
    % transformed = R1to2 * V{1} + t1to2 * ones(1, size(V{1}, 2));
    transformation = matrixForm(R1to2, t1to2);
    accum_matrix = accum_matrix * transformation; % accumulate pose

    flattenedMatrix = reshape(accum_matrix.', 1, []);
    flattenedPose = flattenedMatrix(1:12);

    % Convert the flattened pose to a string with values separated by spaces
    poseString = sprintf('%.6f ', flattenedPose);
    poseString = strtrim(poseString);

    % Write the pose string to the output file
    fprintf(fid, '%s\n', poseString);
end

% Close the output file
fclose(fid);

% Display a message indicating that the poses have been saved
fprintf('Estimated poses have been saved to %s', outputFilePath);

% figure;
% scatter3(transformed(1,:), transformed(2,:), transformed(3,:), 7, 'filled');
% hold on;
% grid on;
% scatter3(V{2}(1,:), V{2}(2,:), V{2}(3,:), 7, 'filled');
% hold on;
% grid on;
% title('Registered LiDAR Scans');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% view(3);