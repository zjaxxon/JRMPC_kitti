function [] = alignPlot(V, R, t)
    % Initialize the cell array TV to store transformed scans
    TV = cell(size(V));
    
    % Loop through each LiDAR scan and apply the estimated transformation
    for j = 1:numel(V)
        % Extract the LiDAR scan for view j
        originalScan = V{j};
        
        % Apply the estimated rotation and translation to the scan
        transformedScan = R{j} * originalScan + t{j} * ones(1, size(originalScan, 2));
        
        % Store the transformed scan in the TV cell array
        TV{j} = transformedScan;
    end
    
    % visualize the final result of the registration.
    fprintf('Ploting....\n');
    
    % Create a new figure for visualization
    % -----------------------------------------------
    % -----------------------------------------------
    figure;
    
    % Loop through each registered LiDAR scan
    for j = 1:numel(TV)
        % Extract the registered LiDAR scan for view j
        registeredScan = TV{j};
    
        % Plot the registered scan
        scatter3(registeredScan(1,:), registeredScan(2,:), registeredScan(3,:), 7, 'filled');
    
        % Adjust plot properties as needed
        hold on;
        grid on;
        title('Registered LiDAR Scans');
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end
    
    % Customize the view angle if desired
    view(3);
    % -----------------------------------------------
    % -----------------------------------------------
end