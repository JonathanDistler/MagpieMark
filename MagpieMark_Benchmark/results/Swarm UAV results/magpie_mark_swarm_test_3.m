%% MAGPIE Mark - Swarm Test 3
% Standalone MATLAB script. Reads your CSV from Windows Downloads.
% Makes five paper-ready figures, each as a 300-DPI PNG and vector PDF.
% Figures:
%   1. Logged swarm target RMSE vs time
%   2. Minimum inter-UAV separation vs time
%   3. Per-drone actual and target XY trajectories
%   4. Individual drone position error vs time
%   5. Logged XY and YZ swarm MSE diagnostics vs time
%
% No CSV has been supplied to the assistant; plots use your actual data
% when this script is run locally in MATLAB.

clear; clc; close all;

%% 1. Read CSV and validate columns
csvFile = 'C:\Users\jonat\Downloads\20261019_swarm_test_3.csv';
testNumber = 3;

if ~isfile(csvFile)
    error('CSV not found: %s\nCheck the file path near the top of this script.', csvFile);
end

T = readtable(csvFile, 'VariableNamingRule', 'preserve');
required = ["elapsed_s", "drone_id", "north", "east", "target_n", ...
    "target_e", "error_m", "swarm_target_rmse_m", "swarm_min_separation_m"];
missing = setdiff(required, string(T.Properties.VariableNames));
if ~isempty(missing)
    error('Missing CSV columns: %s', strjoin(cellstr(missing), ', '));
end

% Sort by elapsed time. All numeric fields can also be text in some CSVs.
t = asNumber(T.elapsed_s);
[t, order] = sort(t);
T = T(order, :);

north = asNumber(T.north);
east = asNumber(T.east);
targetN = asNumber(T.target_n);
targetE = asNumber(T.target_e);
positionError = asNumber(T.error_m);
swarmRMSE = asNumber(T.swarm_target_rmse_m);
minSeparation = asNumber(T.swarm_min_separation_m);

droneId = strtrim(string(T.drone_id));
droneId(ismissing(droneId) | droneId == "") = "unknown";
ids = unique(droneId, 'stable');
colors = lines(numel(ids));

% Aggregate swarm metrics over duplicate elapsed-time records: some logs
% repeat a swarm-wide number once per drone at the same timestamp.
[timeRMSE, valueRMSE] = uniqueTimeMedian(t, swarmRMSE);
[timeSep, valueSep] = uniqueTimeMedian(t, minSeparation);

outDir = fileparts(csvFile);
prefix = sprintf('MAGPIE_swarm_test_%d_', testNumber);

% Optional markers identify changes of phase for the first logged drone.
phaseChangeTimes = [];
if ismember('phase', T.Properties.VariableNames) && ~isempty(ids)
    firstDrone = droneId == ids(1) & isfinite(t);
    phases = string(T.phase(firstDrone));
    phaseT = t(firstDrone);
    if numel(phases) > 1
        changed = find(phases(2:end) ~= phases(1:end-1)) + 1;
        if numel(changed) <= 15
            phaseChangeTimes = phaseT(changed);
        end
    end
end

%% 2. Swarm target RMSE vs elapsed time
fig1 = figure('Color', 'w', 'Position', [100 100 950 480]);
if ~isempty(valueRMSE)
    plot(timeRMSE, valueRMSE, '-', 'LineWidth', 1.7, ...
        'Color', [0.1 0.35 0.75]);
    hold on;
    markPhases(phaseChangeTimes);
    annotateStats(sprintf('Mean = %.3f m   |   Max = %.3f m', ...
        mean(valueRMSE), max(valueRMSE)));
else
    text(0.5, 0.5, 'No finite swarm_target_rmse_m values', ...
        'Units', 'normalized', 'HorizontalAlignment', 'center');
end
xlabel('Elapsed Time (s)');
ylabel('Swarm Target RMSE (m)');
title(sprintf('MAGPIE Mark - Swarm Test %d: Target Tracking RMSE', testNumber));
formatAxes();
savePaperFigure(fig1, outDir, [prefix 'target_RMSE']);

%% 3. Minimum distance between drones vs elapsed time
fig2 = figure('Color', 'w', 'Position', [120 120 950 480]);
if ~isempty(valueSep)
    plot(timeSep, valueSep, '-', 'LineWidth', 1.7, ...
        'Color', [0.12 0.55 0.42]);
    hold on;
    markPhases(phaseChangeTimes);
    annotateStats(sprintf('Minimum = %.3f m   |   Median = %.3f m', ...
        min(valueSep), median(valueSep)));
else
    text(0.5, 0.5, 'No finite swarm_min_separation_m values', ...
        'Units', 'normalized', 'HorizontalAlignment', 'center');
end
xlabel('Elapsed Time (s)');
ylabel('Minimum Inter-Drone Separation (m)');
title(sprintf('MAGPIE Mark - Swarm Test %d: Minimum Separation', testNumber));
formatAxes();
savePaperFigure(fig2, outDir, [prefix 'min_separation']);

%% 4. Individual XY trajectories: actual (solid) vs commanded (dashed)
fig3 = figure('Color', 'w', 'Position', [140 140 830 690]);
hold on;
for k = 1:numel(ids)
    sel = droneId == ids(k) & isfinite(t);
    c = colors(k, :);
    pale = 0.45*c + 0.55*[1 1 1];

    actualSel = sel & isfinite(east) & isfinite(north);
    cmdSel = sel & isfinite(targetE) & isfinite(targetN);

    if any(cmdSel)
        plot(targetE(cmdSel), targetN(cmdSel), '--', ...
            'Color', pale, 'LineWidth', 1.6, ...
            'DisplayName', sprintf('Drone %s: commanded', char(ids(k))));
        % Mark a manageable subset of commanded waypoint changes.
        tx = targetE(cmdSel); ty = targetN(cmdSel);
        idx = find([true; hypot(diff(tx), diff(ty)) > 1e-4]);
        if numel(idx) > 40
            idx = idx(unique(round(linspace(1, numel(idx), 40))));
        end
        plot(tx(idx), ty(idx), 'o', 'Color', pale, 'MarkerSize', 5, ...
            'MarkerFaceColor', pale, 'HandleVisibility', 'off');
    end

    if any(actualSel)
        xx = east(actualSel); yy = north(actualSel);
        plot(xx, yy, '-', 'Color', c, 'LineWidth', 1.8, ...
            'DisplayName', sprintf('Drone %s: actual', char(ids(k))));
        plot(xx(1), yy(1), 's', 'Color', c, 'MarkerFaceColor', c, ...
            'MarkerSize', 7, 'HandleVisibility', 'off');
        plot(xx(end), yy(end), 'd', 'Color', c, 'MarkerFaceColor', c, ...
            'MarkerSize', 7, 'HandleVisibility', 'off');
    end
end
xlabel('East (m)'); ylabel('North (m)');
title(sprintf('MAGPIE Mark - Swarm Test %d: XY Trajectories', testNumber));
axis equal;
formatAxes();
legend('Location', 'bestoutside', 'FontSize', 9);
savePaperFigure(fig3, outDir, [prefix 'XY_trajectory']);

%% 5. Individual UAV position error vs elapsed time
fig4 = figure('Color', 'w', 'Position', [160 160 950 500]);
hold on;
for k = 1:numel(ids)
    good = droneId == ids(k) & isfinite(t) & isfinite(positionError);
    if any(good)
        plot(t(good), positionError(good), '-', ...
            'Color', colors(k, :), 'LineWidth', 1.45, ...
            'DisplayName', sprintf('Drone %s', char(ids(k))));
    end
end
markPhases(phaseChangeTimes);
xlabel('Elapsed Time (s)');
ylabel('Position Error (m)');
title(sprintf('MAGPIE Mark - Swarm Test %d: Individual UAV Error', testNumber));
formatAxes();
legend('Location', 'best', 'FontSize', 10);
savePaperFigure(fig4, outDir, [prefix 'per_drone_error']);

%% 6. Optional XY and YZ mean-square-error diagnostics
if all(ismember(["swarm_xy_mse_m2", "swarm_yz_mse_m2"], ...
        string(T.Properties.VariableNames)))
    xyMSE = asNumber(T.swarm_xy_mse_m2);
    yzMSE = asNumber(T.swarm_yz_mse_m2);
    [timeXY, valueXY] = uniqueTimeMedian(t, xyMSE);
    [timeYZ, valueYZ] = uniqueTimeMedian(t, yzMSE);

    fig5 = figure('Color', 'w', 'Position', [180 180 950 490]);
    hold on;
    if ~isempty(timeXY)
        plot(timeXY, valueXY, '-', 'Color', [0.25 0.42 0.85], ...
            'LineWidth', 1.6, 'DisplayName', 'Logged XY MSE');
    end
    if ~isempty(timeYZ)
        plot(timeYZ, valueYZ, '-', 'Color', [0.85 0.35 0.2], ...
            'LineWidth', 1.6, 'DisplayName', 'Logged YZ MSE');
    end
    markPhases(phaseChangeTimes);
    xlabel('Elapsed Time (s)');
    ylabel('Mean Squared Error (m^2)');
    title(sprintf('MAGPIE Mark - Swarm Test %d: Plane MSE Diagnostics', testNumber));
    formatAxes();
    if ~isempty(timeXY) || ~isempty(timeYZ)
        legend('Location', 'best');
    end
    savePaperFigure(fig5, outDir, [prefix 'plane_MSE']);
else
    fprintf('XY/YZ MSE fields missing: skipping the optional fifth plot.\n');
end

%% 7. Print summary statistics in MATLAB command window
fprintf('\n=== MAGPIE Mark: Swarm Test %d ===\n', testNumber);
fprintf('Input file: %s\n', csvFile);
fprintf('Logged drones: %s\n', strjoin(cellstr(ids), ', '));
if ~isempty(valueRMSE)
    fprintf('Mean reported swarm target RMSE: %.4f m\n', mean(valueRMSE));
    fprintf('Maximum reported swarm target RMSE: %.4f m\n', max(valueRMSE));
end
if ~isempty(valueSep)
    fprintf('Minimum recorded drone separation: %.4f m\n', min(valueSep));
    fprintf('Median recorded drone separation: %.4f m\n', median(valueSep));
end
for k = 1:numel(ids)
    good = droneId == ids(k) & isfinite(positionError) & isfinite(t);
    if any(good)
        droneRMSE = sqrt(mean(positionError(good).^2));
        fprintf('Drone %s: position-error RMS = %.4f m (%d samples)\n', ...
            char(ids(k)), droneRMSE, nnz(good));
    end
end
fprintf('Figures saved to: %s\n', outDir);
fprintf('NOTE: RMSE/MSE interpretation depends on the logger definitions.\n');
fprintf('Waypoint changes may cause spikes in target-relative error.\n\n');

%% Local utility functions (available in MATLAB R2016b and later)
function x = asNumber(col)
    if isnumeric(col) || islogical(col)
        x = double(col(:));
    else
        x = str2double(string(col(:)));
    end
end

function [timeOut, metricOut] = uniqueTimeMedian(times, metric)
    valid = isfinite(times) & isfinite(metric);
    if ~any(valid)
        timeOut = []; metricOut = [];
        return;
    end
    % Median removes duplicates logged by multiple drones at identical times.
    [timeOut, ~, bin] = unique(times(valid));
    metricOut = accumarray(bin, metric(valid), [], @median);
end

function markPhases(changeTimes)
    for j = 1:numel(changeTimes)
        xline(changeTimes(j), ':', 'Color', [0.6 0.6 0.6], ...
            'LineWidth', 0.85, 'HandleVisibility', 'off');
    end
end

function formatAxes()
    ax = gca;
    ax.FontName = 'Arial';
    ax.FontSize = 11;
    ax.LineWidth = 0.9;
    grid on; box on;
end

function annotateStats(labelText)
    text(0.98, 0.96, labelText, 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontSize', 10, 'BackgroundColor', 'white', ...
        'EdgeColor', [0.75 0.75 0.75], 'Margin', 5);
end

function savePaperFigure(fig, outputFolder, stem)
    pngFile = fullfile(outputFolder, [stem '.png']);
    pdfFile = fullfile(outputFolder, [stem '.pdf']);
    exportgraphics(fig, pngFile, 'Resolution', 300);
    try
        exportgraphics(fig, pdfFile, 'ContentType', 'vector');
    catch ME
        warning('Vector PDF export failed for %s: %s', stem, ME.message);
    end
    fprintf('Saved: %s\n', pngFile);
end
