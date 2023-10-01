function [tm] = matrixForm(R, t)
    tm = eye(4);
    tm(1:3, 1:3) = R;
    tm(1:3, 4) = t;
    flattenedMatrix = reshape(tm.', 1, []);
    flattenedPose = flattenedMatrix(1:12);

    % Convert the flattened pose to a string with values separated by spaces
    poseString = sprintf('%.6f ', flattenedPose);
    poseString = strtrim(poseString);

    % Write the pose string to the output file
    fprintf('Transform: %s\n', poseString);
end