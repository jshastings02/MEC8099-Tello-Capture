% Tello Swarm Controller
% Author: Iline Elizebeth Shaju
% Newcastle University Stage 4 MEng Group Project
% Function :GUI Controller for the Aerial Drones Project 
% Updated with: Improved UI layout, controller redesign, text visibility fixes

function telloSwarmControl()
    % Step 1: Set up initial 3D plot with drones and target
    dronePositions = [
        0.5, 0.5, 1;   % Drone 1
       -0.5, 0.5, 1;   % Drone 2
       -0.5, -0.5, 1;  % Drone 3
        0.5, -0.5, 1;  % Drone 4
    ];
    targetPosition = [0, 0, 3];  % Initial target position elevated at Z = 3

    % Default parameters
    plotSize = 4;
    netWidth = 1.0;  % Default net width (meters)
    netHeight = 1.0; % Default net height (meters)

    % Create main figure with improved styling
    fig = uifigure('Name', 'Tello Swarm Control - Technical Dashboard', ...
                  'Position', [100, 100, 1100, 800], ...
                  'Color', [0.96 0.96 0.96], ...
                  'Scrollable', 'on');

    % Create axes for the 3D plot with technical styling (smaller size)
    ax = uiaxes(fig, 'Position', [50, 250, 700, 500], ... % Reduced width from 800 to 700
                'FontName', 'Consolas', ...
                'FontSize', 10, ...
                'XColor', [0.2 0.2 0.2], ...
                'YColor', [0.2 0.2 0.2], ...
                'ZColor', [0.2 0.2 0.2], ...
                'GridColor', [0.8 0.8 0.8], ...
                'Box', 'on');
    ax.XLabel.String = 'X Position (m)';
    ax.YLabel.String = 'Y Position (m)';
    ax.ZLabel.String = 'Z Position (m)';
    title(ax, sprintf('Tello Swarm 3D Visualization | Plot Size: %dm | Net: %.1fm x %.1fm', plotSize, netWidth, netHeight));
    grid(ax, 'on');
    view(ax, 3);
    hold(ax, 'on');
    axis(ax, [-plotSize/2 plotSize/2 -plotSize/2 plotSize/2 0 plotSize]);
    fig.UserData.ax = ax;

    % Plot the target as a red cross
    targetScatter = scatter3(ax, targetPosition(1), targetPosition(2), targetPosition(3), ...
                            150, 'r', 'x', 'LineWidth', 2);

    % Plot the drones in blue (leader in red)
    droneScatter = scatter3(ax, dronePositions(:,1), dronePositions(:,2), dronePositions(:,3), ...
                           150, 'b', 'filled');
    droneScatter.CData(1, :) = [1, 0, 0];  % Drone 1 is the leader (red)

    % Plot the net connecting drones
    netLines = gobjects(4, 1);
    for i = 1:4
        nextIndex = mod(i, 4) + 1;
        netLines(i) = plot3(ax, [dronePositions(i,1), dronePositions(nextIndex,1)], ...
                            [dronePositions(i,2), dronePositions(nextIndex,2)], ...
                            [dronePositions(i,3), dronePositions(nextIndex,3)], ...
                            'k-', 'LineWidth', 2);
    end

    % Initialize path tracking
    pathHistory = cell(4,1);
    for i = 1:4
        pathHistory{i} = dronePositions(i,:);
    end
    pathLines = gobjects(4,1);
    colors = lines(4);
    for i = 1:4
        pathLines(i) = plot3(ax, dronePositions(i,1), dronePositions(i,2), dronePositions(i,3), ...
                       'Color', colors(i,:), 'LineWidth', 1, 'Visible', 'off');
    end

    % Store positions and objects in figure for easy access
fig.UserData.dronePositions = dronePositions;
fig.UserData.targetPosition = targetPosition;

% Movement parameters (updated for accurate speed control)
fig.UserData.stepSize = 0.1;  % Will be updated by updateSpeed()
fig.UserData.rotationAngle = pi/18;  % 10 degrees in radians
fig.UserData.speed = 1.0;  % Default to 1 m/s (realistic medium speed)
fig.UserData.updatesPerSecond = 30;  % Smooth update rate (30 FPS)
fig.UserData.lastMoveTime = posixtime(datetime('now'));  % Precise timing
fig.UserData.lastUpdateTime = now;  % Backup timing

% Visualization handles
fig.UserData.droneScatter = droneScatter;
fig.UserData.targetScatter = targetScatter;
fig.UserData.netLines = netLines;

% Environment parameters
fig.UserData.plotSize = plotSize;
fig.UserData.netWidth = netWidth;
fig.UserData.netHeight = netHeight;

% State tracking
fig.UserData.isMoving = false;
fig.UserData.currentDirection = '';
fig.UserData.isCapturing = false;

% Formation control
fig.UserData.leaderIndex = 1;
fig.UserData.flockingEnabled = false;
fig.UserData.leaderFollowerEnabled = false;
fig.UserData.collisionAvoidanceEnabled = false;
fig.UserData.formationType = 'Square';

% Path visualization
fig.UserData.pathHistory = pathHistory;
fig.UserData.pathLines = pathLines;
fig.UserData.showPaths = false;

% Metrics tracking
fig.UserData.totalDistance = zeros(4,1);  % Meters moved per drone
fig.UserData.movementTime = zeros(4,1);   % Seconds moving per drone
fig.UserData.idleTime = zeros(4,1);       % Seconds idle per drone

% Obstacle management
fig.UserData.obstaclePositions = [];
fig.UserData.obstacleHandles = [];

% Coverage visualization
fig.UserData.coverageOn = false;
fig.UserData.coverageHandle = [];
fig.UserData.coverageText = [];  % Will be created when first used

% Flocking system (optimized parameters)
fig.UserData.velocities = zeros(size(dronePositions));  % For smooth movement
fig.UserData.flockParams = struct(...
    'separationDist', 0.9, ...   % Meters (ideal drone spacing)
    'neighborDist', 1.8, ...     % Meters (flocking influence radius)
    'maxSpeed', 2.0, ...         % m/s (max flocking speed)
    'separationForce', 0.5, ...  % Strength of repulsion
    'cohesionForce', 0.3, ...    % Strength of attraction
    'alignmentForce', 0.4, ...   % Strength of alignment
    'randomness', 0.05, ...      % Organic variation
    'damping', 0.15 ...          % Movement smoothing
);

% Initialize speed display (important for accurate speed control)
updateSpeed(fig, fig.UserData.speed);  % Sets stepSize and updates display
    % Create control panels with improved layout
    createControlPanels(fig);
end

%%%%%%
function createControlPanels(fig)
    % Panel for plot and net configuration
    configPanel = uipanel(fig, 'Title', 'Configuration', ...
                         'Position', [800, 590, 250, 190], ...
                         'BackgroundColor', [0.9 0.9 0.95], ...
                         'FontName', 'Consolas', 'FontWeight', 'bold', ...
                         'FontSize', 10);

    uicontrol(configPanel, 'Style', 'text', 'String', 'Plot Size (m):', ...
              'Position', [10, 125, 100, 20], 'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);
    uicontrol(configPanel, 'Style', 'edit', 'String', num2str(fig.UserData.plotSize), ...
              'Position', [120, 125, 50, 25], 'FontName', 'Consolas', 'FontSize', 9, ...
              'Callback', @(src,~) updatePlotSize(fig, str2double(src.String)));

    uicontrol(configPanel, 'Style', 'text', 'String', 'Net Width (m):', ...
              'Position', [10, 95, 100, 20], 'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);
    uicontrol(configPanel, 'Style', 'edit', 'String', num2str(fig.UserData.netWidth), ...
              'Position', [120, 95, 50, 25], 'FontName', 'Consolas', 'FontSize', 9, ...
              'Callback', @(src,~) updateNetSize(fig, 'width', str2double(src.String)));

    uicontrol(configPanel, 'Style', 'text', 'String', 'Net Height (m):', ...
              'Position', [10, 65, 100, 20], 'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);
    uicontrol(configPanel, 'Style', 'edit', 'String', num2str(fig.UserData.netHeight), ...
              'Position', [120, 65, 50, 25], 'FontName', 'Consolas', 'FontSize', 9, ...
              'Callback', @(src,~) updateNetSize(fig, 'height', str2double(src.String)));

    uicontrol(configPanel, 'Style', 'text', 'String', 'Speed (m/s):', ...
              'Position', [10, 35, 100, 20], 'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);
    speedSlider = uicontrol(configPanel, 'Style', 'slider', ...
                          'Min', 0.01, 'Max', 8, 'Value', fig.UserData.speed, ...
                          'Position', [10, 20, 100, 15], ...
                          'Callback', @(src,~) updateSpeed(fig, src.Value));
    fig.UserData.speedText = uicontrol(configPanel, 'Style', 'text', ...
                                      'String', sprintf('%.2f m/s', fig.UserData.speed), ...
                                      'Position', [120, 20, 50, 15], ...
                                      'BackgroundColor', [0.9 0.9 0.95], ...
                                      'FontName', 'Consolas', 'FontSize', 9);
    
    metricsPanel = uipanel(fig, 'Title', 'Drone Telemetry', ...
            'Position', [800, 355, 250, 220], ...
            'BackgroundColor', [0.9 0.9 0.95], ...
            'FontName', 'Consolas', 'FontWeight', 'bold', ...
            'FontSize', 10, ...
            'Clipping', 'off');   % <<< ADD THIS


    fig.UserData.distanceText = uicontrol(metricsPanel, 'Style', 'text', ...
    'String', 'Distance: D1:0.00 D2:0.00 D3:0.00 D4:0.00', ...
    'Position', [10, 165, 230, 30], 'HorizontalAlignment', 'left', ...
    'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);

    fig.UserData.efficiencyText = uicontrol(metricsPanel, 'Style', 'text', ...
        'String', 'Efficiency: 0% (Tello flight time: 13min)', ...
        'Position', [10, 130, 230, 30], 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);

    fig.UserData.formationText = uicontrol(metricsPanel, 'Style', 'text', ...
        'String', 'Formation: Square', ...
        'Position', [10, 105, 230, 20], 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);

    fig.UserData.statusText = uicontrol(metricsPanel, 'Style', 'text', ...
        'String', 'Status: Idle', ...
        'Position', [10, 85, 230, 20], 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);

    fig.UserData.batteryText = uicontrol(metricsPanel, 'Style', 'text', ...
        'String', 'Battery: Simulated (Tello 1100mAh)', ...
        'Position', [10, 55, 230, 30], 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.9 0.9 0.95], 'FontName', 'Consolas', 'FontSize', 9);
  
  fig.UserData.coverageText = uicontrol(fig, 'Style', 'text', ...
    'String', '', ...
    'Position', [810, 370, 220, 30], ... % just below Drone Telemetry
    'HorizontalAlignment', 'left', ...
    'BackgroundColor', [0.9 0.9 0.95], ...
    'FontName', 'Consolas', 'FontSize', 9);



   % === CONTROLLER PANEL (with refreshed styling) ===
controllerPanel = uipanel(fig, 'Title', 'Drone Controller', ...
    'Position', [50, 15, 720, 220], ...
    'BackgroundColor', [0.90 0.97 0.90], ...  % Soft green shade
    'FontName', 'Consolas', ...
    'FontWeight', 'bold', 'FontSize', 10);

% Movement (arrow keys style)
createButton(fig, controllerPanel, '↑', [90, 140, 60, 40], 'forward', [0 0.7 0]);
createButton(fig, controllerPanel, '←', [20, 110, 60, 40], 'left', [0 0.7 0]);
createButton(fig, controllerPanel, '→', [160, 110, 60, 40], 'right', [0 0.7 0]);
createButton(fig, controllerPanel, '↓', [90, 80, 60, 40], 'back', [0 0.7 0]);

% Z Movement (made smaller to match others)
createButton(fig, controllerPanel, '↑ Z', [260, 140, 60, 40], 'up', [0 0.5 0.8], 'w');
createButton(fig, controllerPanel, '↓ Z', [260, 80, 60, 40], 'down', [0 0.5 0.8], 'w');

% Rotation (also made smaller)
createButton(fig, controllerPanel, '↻', [360, 140, 60, 40], 'clockwise', [0.6 0.6 0.9], 'w');
createButton(fig, controllerPanel, '↺', [360, 80, 60, 40], 'counterclockwise', [0.6 0.6 0.9], 'w');

createButton(fig, controllerPanel, '⏹ STOP', [100, 20, 100, 30], 'stop', [0.8 0 0], 'w');
createButton(fig, controllerPanel, '↺ RESET', [230, 20, 100, 30], 'reset', [0.5 0 0.5], 'w');

% Swarm control buttons
createButton(fig, controllerPanel, 'Flocking', [490, 160, 100, 30], 'toggleFlocking', [0.5 0.5 0.7], 'w');
createButton(fig, controllerPanel, 'Lead-Follow', [490, 125, 100, 30], 'toggleLeaderFollower', [0.5 0.5 0.7], 'w');
createButton(fig, controllerPanel, 'Switch Leader', [600, 160, 100, 30], 'switchLeader', [0.5 0.5 0.7], 'w');
createButton(fig, controllerPanel, 'Avoid Object', [600, 125, 100, 30], 'toggleCollisionAvoidance', [0.5 0.5 0.7], 'w');
createButton(fig, controllerPanel, 'Import Flight Data', [490, 20, 200, 30], 'exportFlightData', [0.7 0.3 0.7]);
% Safety controls

createButton(fig, controllerPanel, 'Leader Fail', [490, 70, 200, 30], 'leaderFail', [0.9 0 0], 'w');

    % === SIMULATION PANEL ===
    simulationPanel = uipanel(fig, 'Title', 'Simulation Features', ...
                              'Position', [800, 15, 250, 240], ...
                              'BackgroundColor', [0.98 0.98 0.98], ...
                              'FontName', 'Consolas', 'FontWeight', 'bold', 'FontSize', 10);

    createButton(fig, simulationPanel, 'Move Target', [20, 180, 200, 30], 'moveTarget', [0.8 0.2 0]);
    createButton(fig, simulationPanel, 'Capture', [20, 140, 200, 30], 'capture', [0.8 0.2 0]);
    createButton(fig, simulationPanel, 'Add Obstacle', [20, 100, 200, 30], 'addObstacle', [0.9 0.4 0]);
    createButton(fig, simulationPanel, 'Show Paths', [20, 60, 200, 30], 'togglePaths', [0.9 0.4 0]);
    createButton(fig, simulationPanel, 'Net Coverage', [20, 20, 200, 30], 'netCoverage', [0.6 0.6 0], 'w');
  

    fig.UserData.flockingPanel = uipanel(fig, 'Title', 'Flocking Forces', ...
    'Position', [800, 270, 250, 75], ...
    'BackgroundColor', [0.9 0.95 0.9], ...
    'FontName', 'Consolas', 'FontWeight', 'bold', 'FontSize', 10, ...
    'Visible', 'off');  % Initially hidden

    fig.UserData.cohesionText = uicontrol(fig.UserData.flockingPanel, 'Style', 'text', ...
    'String', 'Cohesion: 0.00', ...
    'Position', [10, 35, 230, 20], ...
    'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', [0.9 0.95 0.9]);

    fig.UserData.separationText = uicontrol(fig.UserData.flockingPanel, 'Style', 'text', ...
    'String', 'Separation: 0.00', ...
    'Position', [10, 20, 230, 20], ...
    'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', [0.9 0.95 0.9]);

   fig.UserData.alignmentText = uicontrol(fig.UserData.flockingPanel, 'Style', 'text', ...
    'String', 'Alignment: 0.00', ...
    'Position', [10, 5, 230, 20], ...
    'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', [0.9 0.95 0.9]);


    updateMetrics(fig);
end

function createButton(fig, parent, label, position, callback, color, textColor)
    if nargin < 7, textColor = 'k'; end
    uicontrol(parent, 'Style', 'pushbutton', 'String', label, ...
              'Position', position, 'BackgroundColor', color, ...
              'ForegroundColor', textColor, 'FontName', 'Consolas', ...
              'FontWeight', 'bold', 'Callback', @(src,~) buttonCallback(fig, callback));
end


function buttonCallback(fig, action)
    % Ensure the figure is valid before performing any actions
    if isvalid(fig)
        switch action
            case 'forward', startMovement(fig, 'forward');
            case 'back', startMovement(fig, 'back');
            case 'left', startMovement(fig, 'left');
            case 'right', startMovement(fig, 'right');
            case 'up', startMovement(fig, 'up');
            case 'down', startMovement(fig, 'down');
            case 'clockwise', startMovement(fig, 'clockwise');
            case 'counterclockwise', startMovement(fig, 'counterclockwise');
            case 'toggleFlocking', toggleFlocking(fig);
            case 'toggleLeaderFollower', toggleLeaderFollower(fig);
            case 'switchLeader', switchLeader(fig);
            case 'toggleCollisionAvoidance', toggleCollisionAvoidance(fig);
            case 'moveTarget', moveTargetRandomly(fig);
            case 'capture', captureTarget(fig);
            case 'addObstacle', addObstacle(fig);
            case 'togglePaths', togglePaths(fig);
            case 'stop', stopMovement(fig);
            case 'reset', resetSimulation(fig);
            case 'leaderFail', simulateLeaderFailure(fig);
            case 'batterySim', simulateBattery(fig);
            case 'netCoverage', netCoverage(fig);  
            case 'exportFlightData', exportFlightData(fig);

        end
    else
        disp('The figure is no longer valid. Action aborted.');
    end
end


function updatePlotSize(fig, newSize)
    if isnan(newSize) || newSize <= 0
        disp('Invalid plot size. Please enter a positive number.');
        return;
    end
    fig.UserData.plotSize = newSize;
    ax = fig.UserData.ax;
    axis(ax, [-newSize/2 newSize/2 -newSize/2 newSize/2 0 newSize]);
    title(ax, sprintf('Tello Swarm 3D Visualization | Plot Size: %dm | Net: %.1fm x %.1fm', ...
          newSize, fig.UserData.netWidth, fig.UserData.netHeight));
end

function updateNetSize(fig, dimension, value)
    if strcmpi(dimension, 'width')
        fig.UserData.netWidth = value;
    else
        fig.UserData.netHeight = value;
    end
    
    % Update drone positions based on new net size
    leaderPos = fig.UserData.dronePositions(fig.UserData.leaderIndex, :);
    offsets = [
        fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0;
        fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0
    ];
    
    for i = 1:4
        if i ~= fig.UserData.leaderIndex || ~fig.UserData.leaderFollowerEnabled
            fig.UserData.dronePositions(i, :) = leaderPos + offsets(i, :);
        end
    end
    
    % Update plot title
    ax = fig.UserData.ax;
    title(ax, sprintf('Tello Swarm 3D Visualization | Plot Size: %dm | Net: %.1fm x %.1fm', ...
          fig.UserData.plotSize, fig.UserData.netWidth, fig.UserData.netHeight));
    
    updateDronePlot(fig);
end

function updateSpeed(fig, newSpeed)
    % Validate and set speed (0.1-8 m/s range)
    validatedSpeed = max(0.1, min(8, newSpeed));
    fig.UserData.speed = validatedSpeed;
    fig.UserData.stepSize = validatedSpeed / fig.UserData.updatesPerSecond;
    fig.UserData.speedText.String = sprintf('%.2f m/s', validatedSpeed);
end

function startMovement(fig, direction)
    % Mark the drones as moving
    fig.UserData.isMoving = true;
    fig.UserData.currentDirection = direction;
    
    % Ensure smooth updates and avoid disappearing drones
    while fig.UserData.isMoving && isvalid(fig)
        moveDrones(fig, direction);  % Move drones
        
        % Update the plot but only when needed (prevent flickering)
        drawnow limitrate;  % Smooth out the updates and prevent rapid redraws
        pause(0.05);  % Control update rate to avoid freezing or disappearing drones
    end
end


function stopMovement(fig)
    fig.UserData.isMoving = false;
    try
        stop(fig.UserData.movementTimer);
        delete(fig.UserData.movementTimer);
    catch
    end
    fig.UserData.statusText.String = 'Status: Stopped';
end

function continuousMove(fig)
    while fig.UserData.isMoving && isvalid(fig)
        % Original timing calculation
        currentTime = now;
        timeElapsed = (currentTime - fig.UserData.lastUpdateTime) * 86400;
        fig.UserData.lastUpdateTime = currentTime;
        
        % Original movement code
        moveDrones(fig, fig.UserData.currentDirection);
        
        % Add this for smoother updates:
        drawnow limitrate
        pause(0.01); % Small pause to prevent CPU overload
    end
end

function moveDrones(fig, direction)
    % Retrieve drone positions
    positions = fig.UserData.dronePositions;
    oldPositions = positions;
    stepSize = fig.UserData.stepSize;
    
    % Ensure drones move without resetting the plot (no `cla`, `axis` calls here)
    switch direction
        case 'forward', positions(:, 2) = positions(:, 2) + stepSize;
        case 'back',    positions(:, 2) = positions(:, 2) - stepSize;
        case 'left',    positions(:, 1) = positions(:, 1) - stepSize;
        case 'right',   positions(:, 1) = positions(:, 1) + stepSize;
        case 'up',      positions(:, 3) = positions(:, 3) + stepSize;
        case 'down',    positions(:, 3) = positions(:, 3) - stepSize;
        case 'clockwise',        positions = rotateDrones(positions, fig.UserData.rotationAngle);
        case 'counterclockwise', positions = rotateDrones(positions, -fig.UserData.rotationAngle);
    end
    
    % Apply movement and update positions without resetting the plot
    fig.UserData.dronePositions = positions;
    
    updatePathHistory(fig);  % Make sure paths are tracked per move

    % Update the drone positions on the plot
    updateDronePlot(fig);
end

function updateDronePlot(fig)
    % Avoid resetting the plot (no `cla`, `axis`)
    positions = fig.UserData.dronePositions;
    set(fig.UserData.droneScatter, 'XData', positions(:,1), ...
                                   'YData', positions(:,2), ...
                                   'ZData', positions(:,3));
    
    % Update any other visualization elements, like net lines
    updateNetLines(fig);

    if isfield(fig.UserData, 'updateCoverageInline') && fig.UserData.updateCoverageInline
    updateCoverage(fig);  % Live update net patch with drone movement
    end
end

function updateNetLines(fig)
    positions = fig.UserData.dronePositions;
    for i = 1:4
        nextIndex = mod(i, 4) + 1;
        set(fig.UserData.netLines(i), 'XData', [positions(i,1), positions(nextIndex,1)], ...
                                      'YData', [positions(i,2), positions(nextIndex,2)], ...
                                      'ZData', [positions(i,3), positions(nextIndex,3)]);
    end
end

function newPositions = rotateDrones(positions, angle)
    % Corrected rotation function with proper matrix operations
    center = mean(positions, 1);
    translated = positions - center;
    
    % 3D rotation matrix (Z-axis only for this application)
    rotationMatrix = [cos(angle), -sin(angle), 0;
                     sin(angle),  cos(angle), 0;
                     0,          0,           1];
    
    % Apply rotation to all drones
    rotated = (rotationMatrix * translated')';
    newPositions = rotated + center;
end

function moveTargetRandomly(fig)
    plotSize = fig.UserData.plotSize;
    newPosition = [
        (rand - 0.5) * plotSize, ...
        (rand - 0.5) * plotSize, ...
        rand * plotSize
    ];
    fig.UserData.targetPosition = newPosition;
    set(fig.UserData.targetScatter, 'XData', newPosition(1), ...
                                    'YData', newPosition(2), ...
                                    'ZData', newPosition(3));
    fig.UserData.statusText.String = 'Status: Target Moved';
end

function captureTarget(fig)
    % Start capture logic
    fig.UserData.isCapturing = true;
    fig.UserData.statusText.String = 'Status: Capturing Target';
    
    targetPosition = fig.UserData.targetPosition;
    dronePositions = fig.UserData.dronePositions;
    stepSize = fig.UserData.stepSize;
    captureThreshold = 0.2;  % The capture radius
    
    while fig.UserData.isCapturing && isvalid(fig)
        % Track elapsed time for movement logic
        currentTime = now;
        timeElapsed = (currentTime - fig.UserData.lastUpdateTime) * 86400;
        fig.UserData.movementTime = fig.UserData.movementTime + timeElapsed;
        fig.UserData.lastUpdateTime = currentTime;
        
        % Calculate distance from the target
        formationCenter = mean(dronePositions, 1);
        distanceToTarget = norm(targetPosition - formationCenter);
        
        if distanceToTarget <= captureThreshold
            fig.UserData.statusText.String = 'Status: Target Captured!';
            fig.UserData.isCapturing = false;
            break;  % Stop capturing once the target is within range
        end
        
        % Move drones towards the target
        directionToTarget = (targetPosition - formationCenter) / distanceToTarget;
        oldPositions = dronePositions;
        
        for i = 1:size(dronePositions, 1)
            newPosition = dronePositions(i, :) + directionToTarget * stepSize;
            dronePositions(i, :) = newPosition;
            fig.UserData.totalDistance(i) = fig.UserData.totalDistance(i) + norm(dronePositions(i,:) - oldPositions(i,:));
        end
        
        fig.UserData.dronePositions = dronePositions;
        updateDronePlot(fig);
        updatePathHistory(fig);
        updateMetrics(fig);
        pause(1 / fig.UserData.updatesPerSecond);  % Update at the desired frame rate
    end
    
    % Ensure that the drones stay in place after capture
    fig.UserData.isCapturing = false;
    updateMetrics(fig);  % Update telemetry after capture is completed
    
    % Wait for user input (a movement button) to allow movement again
    fig.UserData.isMoving = false;
end


function applyFlocking(fig)
    positions = fig.UserData.dronePositions;
    dt = 1 / fig.UserData.updatesPerSecond;
    parameters = fig.UserData.flockParams;

    if ~isfield(fig.UserData, 'velocities')
        fig.UserData.velocities = zeros(size(positions));
    end

    newPositions = positions;
    avgCoh = 0;
    avgSep = 0;
    avgAlign = 0;

    for i = 1:size(positions,1)
        sepForce = [0 0 0];
        cohForce = [0 0 0];
        alignForce = [0 0 0];
        neighborCount = 0;

        for j = 1:size(positions,1)
            if i == j, continue; end

            offset = positions(i,:) - positions(j,:);
            dist = norm(offset);

            if dist < parameters.separationDist
                sepForce = sepForce + offset / (dist^2 + 0.1);
            end
            if dist < parameters.neighborDist
                cohForce = cohForce + positions(j,:);
                alignForce = alignForce + (positions(j,:) - positions(i,:));
                neighborCount = neighborCount + 1;
            end
        end

        if neighborCount > 0
            cohForce = (cohForce / neighborCount - positions(i,:)) * parameters.cohesionForce;
            alignForce = (alignForce / neighborCount) * parameters.alignmentForce;
        end

        sepForce = sepForce * parameters.separationForce;
        randForce = (rand(1,3)-0.5) * parameters.randomness;

        acceleration = sepForce + cohForce + alignForce + randForce;
        fig.UserData.velocities(i,:) = fig.UserData.velocities(i,:) * (1 - parameters.damping) + acceleration * dt;

        currentSpeed = norm(fig.UserData.velocities(i,:));
        if currentSpeed > parameters.maxSpeed
            fig.UserData.velocities(i,:) = fig.UserData.velocities(i,:) / currentSpeed * parameters.maxSpeed;
        end

        newPositions(i,:) = positions(i,:) + fig.UserData.velocities(i,:) * dt;

        % For telemetry
        avgSep = avgSep + norm(sepForce);
        avgCoh = avgCoh + norm(cohForce);
        avgAlign = avgAlign + norm(alignForce);
    end

    fig.UserData.dronePositions = newPositions;

    % === Update telemetry panel with average force magnitudes ===
    fig.UserData.cohesionText.String = sprintf('Cohesion: %.2f', avgCoh / 4);
    fig.UserData.separationText.String = sprintf('Separation: %.2f', avgSep / 4);
    fig.UserData.alignmentText.String = sprintf('Alignment: %.2f', avgAlign / 4);
end


function toggleFlocking(fig)
    % Toggle flocking on or off and provide visual feedback
    
    % Disable leader-follower if it's active
    if fig.UserData.leaderFollowerEnabled
        fig.UserData.leaderFollowerEnabled = false;
        fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);
    end

    % Toggle the flocking state
    fig.UserData.flockingEnabled = ~fig.UserData.flockingEnabled;

    % Update formation text and drone colors
    if fig.UserData.flockingEnabled
        fig.UserData.formationType = 'Flocking';
        fig.UserData.droneScatter.CData = repmat([0.2, 0.6, 1], 4, 1);
        fig.UserData.statusText.String = 'Status: Flocking Enabled - Free Movement';
        fig.UserData.flockingPanel.Visible = 'on';
        startFlockingLoop(fig);

        % Add slight nudge to escape perfectly square position
        for i = 1:4
            fig.UserData.dronePositions(i,:) = fig.UserData.dronePositions(i,:) + (rand(1,3)-0.5)*0.3;
        end

        updateDronePlot(fig);
    else
        fig.UserData.formationType = 'Square';
        fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);
        fig.UserData.droneScatter.CData(fig.UserData.leaderIndex, :) = [1, 0, 0];
        fig.UserData.statusText.String = 'Status: Flocking Disabled';
        fig.UserData.flockingPanel.Visible = 'off';
    end

    fig.UserData.formationText.String = ['Formation: ' fig.UserData.formationType];
end


function toggleLeaderFollower(fig)
    fig.UserData.leaderFollowerEnabled = ~fig.UserData.leaderFollowerEnabled;
    if fig.UserData.leaderFollowerEnabled
       leaderFollower(fig);  % Reposition drones in net
       fig.UserData.formationType = ['Leader-Follower (D' num2str(fig.UserData.leaderIndex) ')'];
        fig.UserData.flockingEnabled = false;
        % Highlight leader in red
        fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);
        fig.UserData.droneScatter.CData(fig.UserData.leaderIndex, :) = [1, 0, 0];
    else
        fig.UserData.formationType = 'Square';
    end
    fig.UserData.formationText.String = ['Formation: ' fig.UserData.formationType];
    
    % Ensure movement remains functional after toggling
    if fig.UserData.isMoving
        startMovement(fig, fig.UserData.currentDirection);  % Continue movement if it's active
    end
end


function switchLeader(fig)
    fig.UserData.leaderIndex = mod(fig.UserData.leaderIndex, 4) + 1;
    if fig.UserData.leaderFollowerEnabled
        fig.UserData.formationType = ['Leader-Follower (D' num2str(fig.UserData.leaderIndex) ')'];
        fig.UserData.formationText.String = ['Formation: ' fig.UserData.formationType];
        % Highlight new leader in red
        fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);
        fig.UserData.droneScatter.CData(fig.UserData.leaderIndex, :) = [1, 0, 0];
    end
end

function toggleCollisionAvoidance(fig)
    fig.UserData.collisionAvoidanceEnabled = ~fig.UserData.collisionAvoidanceEnabled;
    if fig.UserData.collisionAvoidanceEnabled
        fig.UserData.statusText.String = 'Status: Collision Avoidance ON';
    else
        fig.UserData.statusText.String = 'Status: Collision Avoidance OFF';
    end
end
function togglePaths(fig)
    fig.UserData.showPaths = ~fig.UserData.showPaths;
    
    if fig.UserData.showPaths
        % Show the path lines for all drones
        for i = 1:4
            set(fig.UserData.pathLines(i), 'Visible', 'on');
        end
        fig.UserData.statusText.String = 'Status: Showing Paths';
    else
        % Hide the path lines
        for i = 1:4
            set(fig.UserData.pathLines(i), 'Visible', 'off');
        end
        fig.UserData.statusText.String = 'Status: Paths Hidden';
    end
end


function updatePathHistory(fig)
    positions = fig.UserData.dronePositions;
    for i = 1:4
        % Append the new position to the path history for each drone
        fig.UserData.pathHistory{i} = [fig.UserData.pathHistory{i}; positions(i,:)];
        
        % Update the path lines for each drone (visible after toggle)
        set(fig.UserData.pathLines(i), 'XData', fig.UserData.pathHistory{i}(:,1), ...
                                       'YData', fig.UserData.pathHistory{i}(:,2), ...
                                       'ZData', fig.UserData.pathHistory{i}(:,3));
    end
end



function updateMetrics(fig)
    % Update distance display
    totalDist = sum(fig.UserData.totalDistance);
    distStr = sprintf('Distance: %.2fm (D1:%.2f D2:%.2f D3:%.2f D4:%.2f)', ...
        totalDist, fig.UserData.totalDistance(1), fig.UserData.totalDistance(2), ...
        fig.UserData.totalDistance(3), fig.UserData.totalDistance(4));
    fig.UserData.distanceText.String = distStr;
    
    % Update efficiency display (Tello has ~13min flight time)
    totalTime = fig.UserData.movementTime + fig.UserData.idleTime;
    efficiency = sum(fig.UserData.movementTime) / sum(totalTime) * 100;
    if isnan(efficiency), efficiency = 0; end
    fig.UserData.efficiencyText.String = sprintf('Efficiency: %.1f%% moving (Tello spec: 13min flight)', efficiency);
    
    % Update battery simulation (Tello has 1100mAh battery)
    flightTime = sum(totalTime)/60; % in minutes
    batteryPercent = max(0, 100 - (flightTime/13)*100);
    fig.UserData.batteryText.String = sprintf('Battery: %.0f%% remaining (Tello spec: 1100mAh)', batteryPercent);
end

function netCoverage(fig)
    fig.UserData.coverageOn = ~fig.UserData.coverageOn;

    if fig.UserData.coverageOn
        updateCoverage(fig);
        fig.UserData.statusText.String = 'Status: Coverage Displayed';

        % Hook into plot update (by adding coverage update into updateDronePlot)
        fig.UserData.updateCoverageInline = true;
    else
        if isgraphics(fig.UserData.coverageHandle)
            delete(fig.UserData.coverageHandle);
        end
        fig.UserData.coverageHandle = [];

        if isfield(fig.UserData, 'coverageText') && isgraphics(fig.UserData.coverageText)
            fig.UserData.coverageText.String = '';
        end

        fig.UserData.statusText.String = 'Status: Coverage Hidden';
        fig.UserData.updateCoverageInline = false;
    end
end

function updateCoverage(fig)
    positions = fig.UserData.dronePositions;

    % === CLOCKWISE ORDERING (2D) FOR PATCH ===
    center = mean(positions(:,1:2), 1);
    relPos = positions(:,1:2) - center;
    angles = atan2(relPos(:,2), relPos(:,1));
    [~, sortIdx] = sort(angles);  % Clockwise order

    orderedX = positions(sortIdx, 1);
    orderedY = positions(sortIdx, 2);
    orderedZ = positions(sortIdx, 3);

    % Close the polygon
    orderedX(end+1) = orderedX(1);
    orderedY(end+1) = orderedY(1);
    orderedZ(end+1) = orderedZ(1);

    % === DRAW OR UPDATE 3D PATCH ===
    if isempty(fig.UserData.coverageHandle) || ~isvalid(fig.UserData.coverageHandle)
        fig.UserData.coverageHandle = fill3(fig.UserData.ax, ...
            orderedX, orderedY, orderedZ, ...
            [0.2 0.8 0.2], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    else
        set(fig.UserData.coverageHandle, ...
            'XData', orderedX, ...
            'YData', orderedY, ...
            'ZData', orderedZ);
    end

    % === ANALYTICAL METRICS ===

    % 1. Area (2D XY footprint)
    baseArea = polyarea(orderedX(1:end-1), orderedY(1:end-1));

    % 2. Capture Volume (3D estimate using vertical spread)
    heightSpread = std(positions(:,3));
    captureVolume = baseArea * heightSpread;

    % 3. Net Deformation (based on side lengths)
    d1 = norm(positions(1,:) - positions(2,:));
    d2 = norm(positions(2,:) - positions(3,:));
    d3 = norm(positions(3,:) - positions(4,:));
    d4 = norm(positions(4,:) - positions(1,:));
    idealSide = fig.UserData.netWidth;  % Assuming square net
    deviation = mean(abs([d1 d2 d3 d4] - idealSide));
    shapeScore = max(0, 100 - deviation * 100);  % 100 = perfect

    % 4. Symmetry Ratio (based on diagonals)
    diag1 = norm(positions(1,:) - positions(3,:));
    diag2 = norm(positions(2,:) - positions(4,:));
    symmetry = 1 - abs(diag1 - diag2) / max(diag1, diag2);  % 1 = perfect

    % === UPDATE TELEMETRY TEXT ===
    summary = sprintf('Area: %.2f m² | Volume: %.3f m³\nIntegrity: %.1f%% | Symmetry: %.2f', ...
        baseArea, captureVolume, shapeScore, symmetry);

    fig.UserData.coverageText.String = summary;


end


function addObstacle(fig)
    % Ensure there is an obstacle list in fig.UserData
    if ~isfield(fig.UserData, 'obstaclePositions')
        fig.UserData.obstaclePositions = [];  % Initialize empty obstacle position array
    end
    if ~isfield(fig.UserData, 'obstacleHandles')
        fig.UserData.obstacleHandles = [];  % Initialize empty obstacle handle array
    end

    % Add a new obstacle at a random position
    plotSize = fig.UserData.plotSize;
    pos = [(rand - 0.5) * plotSize, (rand - 0.5) * plotSize, rand * plotSize];  % Random position
    
    % Store the obstacle's position
    fig.UserData.obstaclePositions = [fig.UserData.obstaclePositions; pos];
    
    % Create the obstacle in the plot
    obstacleHandle = scatter3(fig.UserData.ax, pos(1), pos(2), pos(3), 200, 'g', 'filled');
    fig.UserData.obstacleHandles = [fig.UserData.obstacleHandles, obstacleHandle];
    
    fig.UserData.statusText.String = sprintf('Status: %d Obstacles Added', size(fig.UserData.obstaclePositions, 1));
end

function avoidCollisions(fig)
    positions = fig.UserData.dronePositions;
    obstacles = fig.UserData.obstaclePositions;  % Get all obstacles
    minDist = 0.5;  % Minimum distance to obstacles (to trigger avoidance)
    avoidanceRadius = 1.5;  % Distance at which drones will start avoiding obstacles
    strength = 10.0;  % Increased strength of the steering force
    turnSharpness = 1.5;  % Controls how sharp the drone turns around obstacles

    % Loop over all drones
    for i = 1:4
        % Initialize a total steering force for the drone
        totalSteeringForce = [0, 0, 0];
        
        % Loop over all obstacles to calculate the avoidance force
        for j = 1:size(obstacles, 1)
            distObs = norm(positions(i,:) - obstacles(j,:));  % Distance from drone to obstacle
            
            if distObs < avoidanceRadius  % Only steer if within the avoidance radius
                % Calculate a repelling force (away from the obstacle)
                forceDirection = (positions(i,:) - obstacles(j,:)) / distObs;  % Direction vector away from obstacle
                
                % Apply the steering force based on the distance to the obstacle
                steeringForce = forceDirection * (strength / distObs);  % Stronger force the closer the drone is
                
                % Apply turn sharpness: increase sharpness of turn based on distance
                steeringForce = steeringForce * turnSharpness;  % Increase sharpness of the turn
                
                % Accumulate steering force to make the drone turn away from the obstacle
                totalSteeringForce = totalSteeringForce + steeringForce;  
            end
        end
        
        % Apply the total steering force to the drone's position
        positions(i,:) = positions(i,:) + totalSteeringForce;  % Move the drone according to the steering force
    end

    fig.UserData.dronePositions = positions;  % Update the positions after applying avoidance behavior
end

function leaderFollower(fig)
    leaderPos = fig.UserData.dronePositions(fig.UserData.leaderIndex, :);
    offsets = [
        fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0;
        fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0
    ];

    % Calculate target positions
    targetPositions = fig.UserData.dronePositions;
    for i = 1:4
        if i ~= fig.UserData.leaderIndex
            targetPositions(i,:) = leaderPos + offsets(i,:);
        end
    end

    % Smooth transition using interpolation
    steps = 30;
    for s = 1:steps
        for i = 1:4
            fig.UserData.dronePositions(i,:) = ...
                fig.UserData.dronePositions(i,:) + ...
                (targetPositions(i,:) - fig.UserData.dronePositions(i,:)) / (steps - s + 1);
        end
        updateDronePlot(fig);
        updatePathHistory(fig);
        pause(0.01);  % smooth frame rate
    end
end


%%%%%%%
function simulateLeaderFailure(fig)
    pos = fig.UserData.dronePositions;
    leader = fig.UserData.leaderIndex;
    newLeader = mod(leader, 4) + 1;
    offsets = [
        fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, fig.UserData.netHeight/2, 0;
       -fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0;
        fig.UserData.netWidth/2, -fig.UserData.netHeight/2, 0
    ];
    
    for i = 1:4
        if i ~= newLeader
            pos(i, :) = pos(newLeader, :) + offsets(i, :);
        end
    end
    
    fig.UserData.dronePositions = pos;
    fig.UserData.leaderIndex = newLeader;
    fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);
    fig.UserData.droneScatter.CData(newLeader, :) = [1, 0, 0];
    
    if fig.UserData.leaderFollowerEnabled
        fig.UserData.formationType = ['Leader-Follower (D' num2str(newLeader) ')'];
        fig.UserData.formationText.String = ['Formation: ' fig.UserData.formationType];
    end
    fig.UserData.statusText.String = ['Status: Leader Failure Simulated (D' num2str(newLeader) ' is new leader)'];
    
    % Update the formation immediately if in leader-follower mode
    if fig.UserData.leaderFollowerEnabled
        leaderFollower(fig);
        updateDronePlot(fig);
    end
end
function startFlockingLoop(fig)
    % Kill old timer if exists
    if isfield(fig.UserData, 'flockingTimer') && isvalid(fig.UserData.flockingTimer)
        stop(fig.UserData.flockingTimer);
        delete(fig.UserData.flockingTimer);
    end

    % Create new flocking loop timer
    t = timer('ExecutionMode', 'fixedRate', ...
              'Period', 0.1, ...
              'BusyMode', 'drop', ...
              'TimerFcn', @(~,~) flockStep(fig));
    fig.UserData.flockingTimer = t;
    start(t);
end

function flockStep(fig)
    if fig.UserData.flockingEnabled && isvalid(fig)
        applyFlocking(fig);
        updateDronePlot(fig);
        updatePathHistory(fig);
    end
end

function startCoverageLoop(fig)
    if isfield(fig.UserData, 'coverageTimer') && isvalid(fig.UserData.coverageTimer)
        stop(fig.UserData.coverageTimer);
        delete(fig.UserData.coverageTimer);
    end

    t = timer('ExecutionMode', 'fixedRate', ...
              'Period', 0.15, ...
              'TimerFcn', @(~,~) updateCoverage(fig));
    fig.UserData.coverageTimer = t;
    start(t);
end

function resetSimulation(fig)
    % === STOP ALL MOVEMENT & TIMERS ===
    stopMovement(fig);

    % Stop and delete flocking timer if running
    if isfield(fig.UserData, 'flockingTimer') && isvalid(fig.UserData.flockingTimer)
        stop(fig.UserData.flockingTimer);
        delete(fig.UserData.flockingTimer);
    end

    % Stop and delete coverage timer if running
    if isfield(fig.UserData, 'coverageTimer') && isvalid(fig.UserData.coverageTimer)
        stop(fig.UserData.coverageTimer);
        delete(fig.UserData.coverageTimer);
    end

    % === RESET DRONE & TARGET POSITIONS ===
    initialDronePositions = [
        0.5, 0.5, 1;
       -0.5, 0.5, 1;
       -0.5, -0.5, 1;
        0.5, -0.5, 1
    ];
    fig.UserData.dronePositions = initialDronePositions;
    fig.UserData.targetPosition = [0, 0, 3];
    set(fig.UserData.targetScatter, 'XData', 0, 'YData', 0, 'ZData', 3);

    % === RESET STATE VARIABLES ===
    fig.UserData.isMoving = false;
    fig.UserData.isCapturing = false;
    fig.UserData.currentDirection = '';
    fig.UserData.flockingEnabled = false;
    fig.UserData.leaderFollowerEnabled = false;
    fig.UserData.collisionAvoidanceEnabled = false;
    fig.UserData.formationType = 'Square';
    fig.UserData.leaderIndex = 1;
    fig.UserData.updateCoverageInline = false;

    if isfield(fig.UserData, 'velocities')
        fig.UserData.velocities = zeros(size(initialDronePositions));
    end

    % === RESET COVERAGE DISPLAY ===
    if isfield(fig.UserData, 'coverageHandle') && isgraphics(fig.UserData.coverageHandle)
        delete(fig.UserData.coverageHandle);
    end
    fig.UserData.coverageHandle = [];

    if isfield(fig.UserData, 'coverageText') && isgraphics(fig.UserData.coverageText)
        delete(fig.UserData.coverageText);
    end
    fig.UserData.coverageText = [];

    % === RESET PATH HISTORY ===
    for i = 1:4
        fig.UserData.pathHistory{i} = initialDronePositions(i,:);
        set(fig.UserData.pathLines(i), 'XData', initialDronePositions(i,1), ...
                                       'YData', initialDronePositions(i,2), ...
                                       'ZData', initialDronePositions(i,3), ...
                                       'Visible', 'off');
    end
    fig.UserData.showPaths = false;

    % === RESET METRICS ===
    fig.UserData.totalDistance = zeros(4,1);
    fig.UserData.movementTime = zeros(4,1);
    fig.UserData.idleTime = zeros(4,1);

    % === RESET DRONE COLORS ===
    fig.UserData.droneScatter.CData = repmat([0, 0, 1], 4, 1);  % All blue
    fig.UserData.droneScatter.CData(1,:) = [1, 0, 0];  % Leader red

    % === REMOVE OBSTACLES ===
    if isfield(fig.UserData, 'obstacleHandles')
        delete(fig.UserData.obstacleHandles(ishandle(fig.UserData.obstacleHandles)));
        fig.UserData.obstacleHandles = [];
        fig.UserData.obstaclePositions = [];
    end

    % === UI FEEDBACK ===
    fig.UserData.formationText.String = 'Formation: Square';
    fig.UserData.statusText.String = 'Status: Reset Complete';
    updateDronePlot(fig);
    updateMetrics(fig);
    drawnow;
end

function exportFlightData(fig)
    timestamp = datestr(now, 'dd-mm-yyyy_HH-MM-SS');
    filename = sprintf('BoidFlight_%s.xlsx', timestamp);

    numDrones = size(fig.UserData.dronePositions, 1);
    history = fig.UserData.pathHistory;

    % === Write drone paths to separate sheets ===
    for i = 1:numDrones
        data = history{i};
        T = array2table(data, 'VariableNames', {'X', 'Y', 'Z'});
        writetable(T, filename, 'Sheet', sprintf('Drone%d_Path', i));
    end

    % === ANALYSIS STATS ===
    pos = fig.UserData.dronePositions;
    area = 0; volume = 0; shapeScore = 0; symmetry = 0; offset = NaN;
    idealSide = fig.UserData.netWidth;

    % 1. Net Deformation
    d1 = norm(pos(1,:) - pos(2,:));
    d2 = norm(pos(2,:) - pos(3,:));
    d3 = norm(pos(3,:) - pos(4,:));
    d4 = norm(pos(4,:) - pos(1,:));
    deviation = mean(abs([d1 d2 d3 d4] - idealSide));
    shapeScore = max(0, 100 - deviation * 100);

    % 2. Symmetry
    diag1 = norm(pos(1,:) - pos(3,:));
    diag2 = norm(pos(2,:) - pos(4,:));
    symmetry = 1 - abs(diag1 - diag2) / max(diag1, diag2);

    % 3. Area & Volume
    area = polyarea(pos(:,1), pos(:,2));
    heightSpread = std(pos(:,3));
    volume = area * heightSpread;

    % 4. Target Centering Offset
    if isfield(fig.UserData, 'targetPosition')
        netCenter = mean(pos);
        offset = norm(netCenter - fig.UserData.targetPosition);
    end

    % 5. Drone Spread (max pairwise distance)
    spread = max(pdist(pos));  % 3D pairwise distance

    % 6. Total Movement Time and Efficiency
    totalMoveTime = sum(fig.UserData.movementTime);
    totalIdleTime = sum(fig.UserData.idleTime);
    flightTime = totalMoveTime + totalIdleTime;
    efficiency = (totalMoveTime / flightTime) * 100;

    % === Build Summary Table ===
    summary = {
        'Net Width (m)', fig.UserData.netWidth;
        'Net Height (m)', fig.UserData.netHeight;
        'Plot Size (m)', fig.UserData.plotSize;
        'Speed (m/s)', fig.UserData.speed;
        'Flight Area (m²)', area;
        'Capture Volume (m³)', volume;
        'Net Integrity (%)', shapeScore;
        'Symmetry Ratio', symmetry;
        'Target Offset (m)', offset;
        'Max Drone Spread (m)', spread;
        'Total Movement Time (s)', totalMoveTime;
        'Total Idle Time (s)', totalIdleTime;
        'Movement Efficiency (%)', efficiency;
        'Distance D1 (m)', fig.UserData.totalDistance(1);
        'Distance D2 (m)', fig.UserData.totalDistance(2);
        'Distance D3 (m)', fig.UserData.totalDistance(3);
        'Distance D4 (m)', fig.UserData.totalDistance(4);
    };

    summaryTable = cell2table(summary(:,2), 'VariableNames', {'Value'});
    summaryTable.Properties.RowNames = summary(:,1);

    writetable(summaryTable, filename, 'Sheet', 'FlightSummary', 'WriteRowNames', true);

    fig.UserData.statusText.String = sprintf('Status: Flight Data Saved → %s', filename);
end

