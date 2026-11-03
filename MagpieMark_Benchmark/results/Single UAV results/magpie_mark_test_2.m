%% MAGPIE Mark — UAV Navigation Performance Figures
% Reads one CSV log and generates two publication-ready figures.
clear; clc; close all;

%% 1. Load data
csvFile = 'C:\Users\jonat\Downloads\20261019_test_2.csv';
T = readtable(csvFile, 'VariableNamingRule', 'preserve');

required = ["elapsed_s", "error_m", "north", "east", "target_n", "target_e"];
missing = setdiff(required, string(T.Properties.VariableNames));
if ~isempty(missing)
    error('Missing CSV columns: %s', strjoin(cellstr(missing), ', '));
end

% Convert numeric-looking text columns to numbers if necessary.
t = numericColumn(T.elapsed_s);
e = numericColumn(T.error_m);
n = numericColumn(T.north);
east = numericColumn(T.east);
targetN = numericColumn(T.target_n);
targetE = numericColumn(T.target_e);

% Sort time-series data so plots progress chronologically.
[~, order] = sort(t);
t = t(order);
e = e(order);
n = n(order);
east = east(order);
targetN = targetN(order);
targetE = targetE(order);

outputDir = fileparts(csvFile);

%% 2. Figure 1 — Position tracking error vs. time
validError = isfinite(t) & isfinite(e);
if ~any(validError)
    error('No valid elapsed_s / error_m samples found.');
end

rmseError = sqrt(mean(e(validError).^2));
meanError = mean(e(validError));
maxError = max(e(validError));

fig1 = figure('Color', 'w', 'Position', [100 100 950 500]);
plot(t(validError), e(validError), 'b-', 'LineWidth', 1.5);
hold on;

% Add waypoint-change markers when the waypoint field is available.
if ismember('waypoint', T.Properties.VariableNames)
    waypoint = string(T.waypoint);
    waypoint = waypoint(order);
    changeIndices = find(waypoint(2:end) ~= waypoint(1:end-1)) + 1;
    % Avoid cluttering the plot if the log has many changes.
    if numel(changeIndices) <= 30
        for k = 1:numel(changeIndices)
            idx = changeIndices(k);
            if isfinite(t(idx))
                xline(t(idx), 'k:', 'HandleVisibility', 'off');
            end
        end
    end
end

xlabel('Elapsed Time (s)', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('MAGPIE Mark Test 2: Position Tracking Error', 'FontSize', 14);
grid on; box on;
set(gca, 'FontSize', 11);

stats = sprintf('RMSE = %.3f m\nMean = %.3f m\nMax = %.3f m', ...
    rmseError, meanError, maxError);
text(0.98, 0.97, stats, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', 'EdgeColor', [0.7 0.7 0.7], ...
    'Margin', 6, 'FontSize', 10);

exportgraphics(fig1, fullfile(outputDir, 'MAGPIE_test_2_tracking_error.png'), ...
    'Resolution', 300);

%% 3. Figure 2 — Actual vs. commanded XY trajectory
validXY = isfinite(east) & isfinite(n) & ...
          isfinite(targetE) & isfinite(targetN) & isfinite(t);
if ~any(validXY)
    error('No valid actual/target north-east position samples found.');
end

xActual = east(validXY);
yActual = n(validXY);
xTarget = targetE(validXY);
yTarget = targetN(validXY);

% Keep the first target and subsequent distinct waypoint changes.
waypointChange = [true; hypot(diff(xTarget), diff(yTarget)) > 1e-6];

fig2 = figure('Color', 'w', 'Position', [150 150 750 650]);
plot(xTarget(waypointChange), yTarget(waypointChange), 'ro--', ...
    'LineWidth', 1.5, 'MarkerSize', 7, 'MarkerFaceColor', 'r');
hold on;
plot(xActual, yActual, 'b-', 'LineWidth', 1.5);
plot(xActual(1), yActual(1), 'ks', 'MarkerFaceColor', 'g', ...
    'MarkerSize', 9);
plot(xActual(end), yActual(end), 'kd', 'MarkerFaceColor', 'y', ...
    'MarkerSize', 9);

xlabel('East (m)', 'FontSize', 12);
ylabel('North (m)', 'FontSize', 12);
title('MAGPIE Mark Test 2: Commanded vs. Actual XY Trajectory', 'FontSize', 14);
legend('Commanded Waypoints', 'Actual Trajectory', 'Start', 'End', ...
    'Location', 'best');
axis equal;
grid on; box on;
set(gca, 'FontSize', 11);

exportgraphics(fig2, fullfile(outputDir, 'MAGPIE_test_2_XY_trajectory.png'), ...
    'Resolution', 300);

%% 4. Print quantitative performance metrics
fprintf('\nMAGPIE Mark — Test 2 Navigation Performance\n');
fprintf('CSV: %s\n', csvFile);
fprintf('Valid error samples: %d\n', nnz(validError));
fprintf('Position error RMSE: %.4f m\n', rmseError);
fprintf('Mean position error: %.4f m\n', meanError);
fprintf('Maximum position error: %.4f m\n', maxError);
fprintf('Figures saved in: %s\n', outputDir);

%% Local function: handle either numeric or text columns
function x = numericColumn(v)
    if isnumeric(v) || islogical(v)
        x = double(v);
    else
        x = str2double(string(v));
    end
    x = x(:);
end
