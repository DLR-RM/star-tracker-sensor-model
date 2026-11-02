clear; clc;

thisFile = mfilename('fullpath');
if isempty(thisFile)
    testDir = pwd;
else
    testDir = fileparts(thisFile);
end
repoRoot = fileparts(testDir);
binDir = fullfile(repoRoot, 'bin');
resultsDir = fullfile(testDir, 'results');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

addpath(testDir);
addpath(binDir);

[osTag, modelName] = selectPlatformModel();
modelFile = fullfile(testDir, [modelName '.mdl']);
if ~exist(modelFile, 'file')
    error('Expected Simulink model not found: %s', modelFile);
end

fprintf('Running FMU tests with model %s on %s.\n', modelName, osTag);
load_system(modelName);
cleanup = onCleanup(@() close_system(modelName, 0));

timeData = runCase(modelName, makeTimeplotCase());
plotEulerTime(timeData, resultsDir, osTag);

statsData = runCase(modelName, makeStatisticsCase());
[attitudeError, covDiag] = calculateAttitudeError(statsData);
plotAttitudeHistogram(attitudeError, resultsDir, osTag);
plotCovariance(statsData.t, covDiag, resultsDir, osTag);

fovCase = makeInFovCase();
fovData = runCase(modelName, fovCase);
plotInFov(fovData, fovCase, resultsDir, osTag);

starData = runCase(modelName, makeUrsaMinorCase());
plotUrsaMinor(starData, resultsDir, osTag);

writeResultsInfo(resultsDir, osTag, modelName, repoRoot, thisFile);

fprintf('Saved FMU test figures to %s.\n', resultsDir);

function [osTag, modelName] = selectPlatformModel()
    if ispc
        osTag = 'win';
        modelName = 'fmu_test_win';
    elseif isunix && ~ismac
        osTag = 'linux';
        modelName = 'fmu_test_linux';
    else
        error('No FMU test model is provided for this operating system.');
    end
end

function c = makeTimeplotCase()
    d2r = pi/180;

    c = defaultCase();
    c.t_sim = 4;
    c.dt = 0.01;
    c.r_in = [0, 0, 0, 1e6; 10, 0, 0, 1e6];
    c.eul_in = [
        0, -0.008,  0.010, -0.001;
        2,  0.000,  0.000,  0.000;
        4,  0.008, -0.010,  0.001] * diag([1, d2r, d2r, d2r]);
end

function c = makeStatisticsCase()
    d2r = pi/180;

    c = defaultCase();
    c.t_sim = 120;
    c.dt = 0.01;
    c.r_in = [0, 0, 0, 1e6; 10, 0, 0, 1e6];
    c.eul_in = [0, 0, 0, 0; c.t_sim, 0, 360, 0] * diag([1, d2r, d2r, d2r]);
end

function c = makeInFovCase()
    d2r = pi/180;

    c = defaultCase();
    c.p.sigmaDist = 1e-4*d2r;
    c.p.moonAvoidanceAngle = 10*d2r;
    c.t_sim = 60;
    c.dt = 0.01;
    c.r_in = [0, 0, 0, 1e8; 10, 0, 0, 1e8];
    c.eulerStartDeg = 45;
    c.eulerStopDeg = 315;
    c.eul_in = [
        0,       0, c.eulerStartDeg, 0;
        c.t_sim, 0, c.eulerStopDeg,  0] * diag([1, d2r, d2r, d2r]);
end

function c = makeUrsaMinorCase()
    d2r = pi/180;

    c = defaultCase();
    c.p.FOVAngle = 20*d2r;
    c.t_sim = 10;
    c.dt = 0.01;
    c.r_in = [0, 0, 0, 1e7; 10, 0, 0, 1e7];
    c.eul_in = [0, 0, 0, 0; c.t_sim, 0, 0, 0];
end

function c = defaultCase()
    d2r = pi/180;
    fixedLocalSeed = int32(13300988);
    fixedGlobalSeed = int32(39941994);

    p = struct();
    p.sigmaDist = 1.745e-05;
    p.muDist = 0;
    p.FOVAngle = 20*d2r;
    p.earthAvoidanceAngle = 20*d2r;
    p.sunAvoidanceAngle = 26*d2r;
    p.moonAvoidanceAngle = 0*d2r;
    p.localSeed = fixedLocalSeed;
    p.globalSeed = fixedGlobalSeed;

    c = struct();
    c.p = p;
    c.t_sim = 1;
    c.dt = 0.01;
    c.r_in = [0, 0, 0, 1e6; 1, 0, 0, 1e6];
    c.eul_in = [0, 0, 0, 0; 1, 0, 0, 0];
end

function data = runCase(modelName, c)
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setVariable('p', c.p);
    simIn = simIn.setVariable('r_in', c.r_in);
    simIn = simIn.setVariable('eul_in', c.eul_in);
    simIn = simIn.setVariable('t_sim', c.t_sim);
    simIn = simIn.setVariable('dt', c.dt);

    out = sim(simIn);

    data = struct();
    data.t = out.tout;
    data.B_e = signalValues(out, 'B_e');
    data.B_s = signalValues(out, 'B_s');
    data.B_m = signalValues(out, 'B_m');
    data.q_in = signalValues(out, 'q_in');
    data.q_est = signalValues(out, 'q_est');
    data.eul_in = signalValues(out, 'eul_in');
    data.visibleStarsECI = signalValues(out, 'visibleStarsECI');
    data.visibleStarsID = signalValues(out, 'visibleStarsID');
    data.cov = signalValues(out, 'Cov');
    data.eul_in_calc = quat2eul(data.q_in, 'XYZ');
    data.eul_est = quat2eul(data.q_est, 'XYZ');
end

function values = signalValues(out, name)
    sig = out.get(name);
    if isstruct(sig) && isfield(sig, 'signals')
        values = sig.signals.values;
    elseif isa(sig, 'timeseries')
        values = sig.Data;
    else
        values = sig;
    end
end

function [attitudeError, covDiag] = calculateAttitudeError(data)
    n = size(data.q_in, 1);
    attitudeError = zeros(n, 3);
    covDiag = zeros(n, 3);
    covRaw = data.cov;

    if ndims(covRaw) == 3
        covDiag = squeeze([covRaw(1, 1, :), covRaw(2, 2, :), covRaw(3, 3, :)])';
    elseif size(covRaw, 2) >= 9
        covDiag = covRaw(:, [1, 5, 9]);
    elseif size(covRaw, 2) >= 3
        covDiag = covRaw(:, 1:3);
    end

    for i = 2:n
        qTrue = data.q_in(i - 1, :);
        qEst = data.q_est(i, :);
        rTrue = quat2rotm(qTrue);
        rEst = quat2rotm(qEst);
        dR = rTrue*rEst';
        dEuler = rotm2eul(dR, 'XYZ');
        attitudeError(i, :) = (rTrue'*dEuler')';
    end

    for i = 1:3
        std_i = std(attitudeError(:, i));
        if std_i > 0
            attitudeError(abs(attitudeError(:, i)) > 10*std_i, i) = 0;
        end
    end
end

function plotEulerTime(data, resultsDir, osTag)
    d2r = pi/180;

    fig = figure('Name', 'Euler angle timeplot', 'Color', 'w');
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    stairs(ax, data.t, 3600*data.eul_est/d2r);
    stairs(ax, data.t, 3600*data.eul_in/d2r, 'k');
    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Euler angles (arcsec)');
    legend(ax, {'\psi_{est}', '\theta_{est}', '\phi_{est}', 'input'}, 'Location', 'best');

    saveFigure(fig, resultsDir, sprintf('euler_angle_timeplot_%s', osTag));
end

function plotAttitudeHistogram(attitudeError, resultsDir, osTag)
    d2r = pi/180;

    fig = figure('Name', 'Attitude error histogram', 'Color', 'w');
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    histogram(ax, attitudeError(:, 3)/d2r*3600, 'FaceAlpha', 1, ...
        'BinWidth', 2, 'Normalization', 'percentage');
    histogram(ax, attitudeError(:, 2)/d2r*3600, 'FaceAlpha', 1, ...
        'BinWidth', 0.2, 'Normalization', 'percentage');
    histogram(ax, attitudeError(:, 1)/d2r*3600, 'FaceAlpha', 1, ...
        'BinWidth', 0.2, 'Normalization', 'percentage');
    xlabel(ax, 'Attitude error (arcsec)');
    ylabel(ax, 'Samples (%)');
    legend(ax, {'\phi', '\theta', '\psi'}, 'Location', 'best');
    xlim(ax, [-30, 30]);

    saveFigure(fig, resultsDir, sprintf('attitude_error_histogram_%s', osTag));
end

function plotCovariance(t, covDiag, resultsDir, osTag)
    fig = figure('Name', 'Covariance timeplot', 'Color', 'w');
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    plot(ax, t, covDiag);
    set(ax, 'YScale', 'log');
    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Covariance (-)');
    legend(ax, {'\sigma_{xx}^2', '\sigma_{yy}^2', '\sigma_{zz}^2'}, 'Location', 'best');

    saveFigure(fig, resultsDir, sprintf('covariance_timeplot_%s', osTag));
end

function plotInFov(data, c, resultsDir, osTag)
    xAxis = linspace(c.eulerStartDeg, c.eulerStopDeg, length(data.t));

    fig = figure('Name', 'In-FOV analysis', 'Color', 'w');
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'off');
    stairs(ax, xAxis, data.B_e, 'b');
    stairs(ax, xAxis, data.B_s, 'r');
    stairs(ax, xAxis, data.B_m, 'k');
    xlabel(ax, '\theta (deg)');
    ylabel(ax, 'In-FOV (true/false)');
    legend(ax, {'Earth', 'Sun', 'Moon'}, 'Location', 'best');
    ylim(ax, [-0.5, 1.5]);
    yticks(ax, [0, 1]);
    xlim(ax, [c.eulerStartDeg, c.eulerStopDeg]);
    xticks(ax, c.eulerStartDeg:45:c.eulerStopDeg);

    saveFigure(fig, resultsDir, sprintf('in_fov_analysis_%s', osTag));
end

function plotUrsaMinor(data, resultsDir, osTag)
    [visibleStars, starIDs] = firstVisibleStarSample(data);
    isStar = sum(abs(visibleStars), 1) > 0;
    visibleStars = visibleStars(:, isStar);
    starIDs = starIDs(isStar);

    ursaMinorHR = [424, 5563, 5735, 6789, 6322, 5903, 6116, ...
        4301, 4295, 4554, 4660, 5054, 4905, 5191];
    isUrsaMinor = ismember(starIDs, ursaMinorHR);
    ursaMinorStars = visibleStars(:, isUrsaMinor);

    fig = figure('Name', 'Visible stars near Ursa Minor', 'Color', 'w');
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    plot(ax, visibleStars(1, :), visibleStars(2, :), '.', 'MarkerSize', 12);
    plot(ax, ursaMinorStars(1, :), ursaMinorStars(2, :), 'o', ...
        'MarkerSize', 10, 'LineWidth', 1.5);
    xlabel(ax, 'v_x (-)');
    ylabel(ax, 'v_y (-)');
    legend(ax, {'Visible stars', 'Ursa Minor HR stars'}, 'Location', 'best');

    saveFigure(fig, resultsDir, sprintf('ursa_minor_star_plot_%s', osTag));
end

function [visibleStars, starIDs] = firstVisibleStarSample(data)
    starsRaw = data.visibleStarsECI;
    idsRaw = data.visibleStarsID;

    if ndims(starsRaw) == 3
        visibleStars = starsRaw(:, :, 1);
    elseif size(starsRaw, 2) == 3
        visibleStars = starsRaw(1, :);
        visibleStars = reshape(visibleStars, 3, []);
    else
        visibleStars = starsRaw;
    end

    if ndims(idsRaw) == 3
        starIDs = squeeze(idsRaw(1, :, 1));
    elseif ismatrix(idsRaw) && size(idsRaw, 1) > 1
        starIDs = idsRaw(1, :);
    else
        starIDs = idsRaw(:)';
    end
end

function saveFigure(fig, resultsDir, baseName)
    pngFile = fullfile(resultsDir, [baseName '.png']);
    exportgraphics(fig, pngFile, 'Resolution', 300);
end

function writeResultsInfo(resultsDir, osTag, modelName, repoRoot, scriptPath)
    fmuPath = platformFmuPath(repoRoot, osTag);
    fmuMeta = readFmuMetadata(fmuPath);

    infoFile = fullfile(resultsDir, sprintf('results_info_%s.txt', osTag));
    fid = fopen(infoFile, 'w');
    if fid < 0
        error('Could not open results info file for writing: %s', infoFile);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, 'Generated: %s\n', char(datetime('now', 'TimeZone', 'local')));
    fprintf(fid, 'OS tag: %s\n', osTag);
    fprintf(fid, 'Simulink model: %s.mdl\n', modelName);
    fprintf(fid, 'Script: %s\n', relativePath(scriptPath, repoRoot));
    fprintf(fid, '\n');
    fprintf(fid, 'FMU file: %s\n', relativePath(fmuPath, repoRoot));
    fprintf(fid, 'FMU SHA-256: %s\n', sha256File(fmuPath));
    fmuFile = dir(fmuPath);
    if ~isempty(fmuFile)
        fprintf(fid, 'FMU size bytes: %.0f\n', fmuFile.bytes);
        fprintf(fid, 'FMU modified: %s\n', fmuFile.date);
    end
    fprintf(fid, 'FMU modelName: %s\n', fmuMeta.modelName);
    fprintf(fid, 'FMU guid: %s\n', fmuMeta.guid);
    fprintf(fid, 'FMU version: %s\n', fmuMeta.version);
    fprintf(fid, 'FMU generationTool: %s\n', fmuMeta.generationTool);
    fprintf(fid, 'FMU generationDateAndTime: %s\n', fmuMeta.generationDateAndTime);
    fprintf(fid, 'FMU fmiVersion: %s\n', fmuMeta.fmiVersion);
    fprintf(fid, 'FMU modelDescription SHA-256: %s\n', fmuMeta.modelDescriptionSha256);
end

function fmuPath = platformFmuPath(repoRoot, osTag)
    switch osTag
        case 'win'
            fmuName = 'startracker_error_model_n16_clk0p1_v140_win.fmu';
        case 'linux'
            fmuName = 'startracker_error_model_n16_clk0p1_v140_linux.fmu';
        otherwise
            error('No FMU mapping is defined for OS tag: %s', osTag);
    end
    fmuPath = fullfile(repoRoot, 'bin', fmuName);
    if ~exist(fmuPath, 'file')
        error('Expected FMU not found: %s', fmuPath);
    end
end

function meta = readFmuMetadata(fmuPath)
    emptyValue = '<unavailable>';
    meta = struct( ...
        'modelName', emptyValue, ...
        'guid', emptyValue, ...
        'version', emptyValue, ...
        'generationTool', emptyValue, ...
        'generationDateAndTime', emptyValue, ...
        'fmiVersion', emptyValue, ...
        'modelDescriptionSha256', emptyValue);

    extractDir = tempname;
    mkdir(extractDir);
    cleanup = onCleanup(@() removeDirectoryIfPresent(extractDir));

    try
        unzip(fmuPath, extractDir);
        modelDescriptionPath = fullfile(extractDir, 'modelDescription.xml');
        if ~exist(modelDescriptionPath, 'file')
            return;
        end

        meta.modelDescriptionSha256 = sha256File(modelDescriptionPath);
        doc = xmlread(modelDescriptionPath);
        root = doc.getDocumentElement();
        meta.modelName = char(root.getAttribute('modelName'));
        meta.guid = char(root.getAttribute('guid'));
        meta.version = char(root.getAttribute('version'));
        meta.generationTool = char(root.getAttribute('generationTool'));
        meta.generationDateAndTime = char(root.getAttribute('generationDateAndTime'));
        meta.fmiVersion = char(root.getAttribute('fmiVersion'));
    catch err
        warning('Could not read FMU metadata from %s: %s', fmuPath, err.message);
    end
end

function hash = sha256File(filePath)
    try
        fid = fopen(filePath, 'rb');
        if fid < 0
            error('Could not open file.');
        end
        cleanup = onCleanup(@() fclose(fid));
        data = fread(fid, inf, '*uint8');
        md = java.security.MessageDigest.getInstance('SHA-256');
        md.update(typecast(data, 'int8'));
        hashBytes = typecast(md.digest(), 'uint8');
        hash = lower(reshape(dec2hex(hashBytes, 2)', 1, []));
    catch err
        warning('Could not compute SHA-256 for %s: %s', filePath, err.message);
        hash = '<unavailable>';
    end
end

function removeDirectoryIfPresent(pathToRemove)
    if exist(pathToRemove, 'dir')
        rmdir(pathToRemove, 's');
    end
end

function relPath = relativePath(pathValue, rootPath)
    if isempty(pathValue)
        relPath = '<unavailable>';
        return;
    end

    pathValue = char(pathValue);
    rootPath = char(rootPath);
    rootWithSeparator = [rootPath filesep];

    if startsWith(pathValue, rootWithSeparator, 'IgnoreCase', ispc)
        relPath = extractAfter(pathValue, strlength(rootWithSeparator));
    elseif strcmpi(pathValue, rootPath)
        relPath = '.';
    else
        [~, name, ext] = fileparts(pathValue);
        relPath = [name ext];
    end

    relPath = char(strrep(relPath, '\', '/'));
end
