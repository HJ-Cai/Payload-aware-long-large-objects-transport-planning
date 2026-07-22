function y = deflection_predictor(diameter_mm, length_m)
%DEFLECTION_PREDICTOR Predict rebar static sag/deflection in metres.
%   y = deflection_predictor(diameter_mm, length_m)
%
%   Inputs:
%       diameter_mm : rebar diameter in millimetres
%       length_m    : effective cantilever length in metres
%
%   The current predictor uses the retrained Keras model:
%       deflection_nn_4x7_bs32_lr001.keras
%       defl_scaler_4x7_bs32_lr001.json
%
%   Feature order is [diameter_m, length_m]. The output is clipped to zero
%   because negative static sag is not physically meaningful.

    arguments
        diameter_mm (1,1) double {mustBeFinite, mustBePositive}
        length_m    (1,1) double {mustBeFinite, mustBePositive}
    end

    persistent MODEL MU SIG
    if isempty(MODEL) || isempty(MU) || isempty(SIG)
        [MODEL, MU, SIG] = i_loadKerasDenseModel();
    end

    x = ([diameter_mm/1000.0, length_m] - MU) ./ SIG;
    a = x;
    for k = 1:(numel(MODEL.W)-1)
        a = max(0, a * MODEL.W{k} + MODEL.b{k});
    end
    y = a * MODEL.W{end} + MODEL.b{end};
    y = max(0, double(y(1)));
end

function [model, mu, sigma] = i_loadKerasDenseModel()
    rootDir = fileparts(mfilename("fullpath"));
    modelPath = fullfile(rootDir, "deflection_nn_4x7_bs32_lr001.keras");
    scalerPath = fullfile(rootDir, "defl_scaler_4x7_bs32_lr001.json");

    if ~isfile(modelPath)
        modelPath = fullfile(rootDir, "final_result_deflection_analysis", ...
            "deflection_nn_4x7_bs32_lr001.keras");
    end
    if ~isfile(scalerPath)
        scalerPath = fullfile(rootDir, "final_result_deflection_analysis", ...
            "defl_scaler_4x7_bs32_lr001.json");
    end

    assert(isfile(modelPath), "Missing Keras deflection model: %s", modelPath);
    assert(isfile(scalerPath), "Missing deflection scaler JSON: %s", scalerPath);

    S = jsondecode(fileread(scalerPath));
    mu = double(S.mean(:)).';
    sigma = double(S.scale(:)).';
    assert(numel(mu)==2 && numel(sigma)==2, ...
        "Scaler JSON must contain two features: [diameter_m, length_m].");

    h5path = i_extractKerasWeights(modelPath);
    layerNames = i_denseLayerStorageNames(h5path);
    assert(numel(layerNames) >= 2, "Expected at least one hidden Dense layer and one output layer.");

    model.W = cell(1, numel(layerNames));
    model.b = cell(1, numel(layerNames));
    for k = 1:numel(layerNames)
        base = "/layers/" + layerNames(k) + "/vars/";
        model.W{k} = double(h5read(h5path, base + "0")).';
        model.b{k} = double(h5read(h5path, base + "1")).';
    end
end

function h5path = i_extractKerasWeights(modelPath)
    [~, stem] = fileparts(modelPath);
    unpackDir = fullfile(tempdir, "rebar_deflection_" + stem);
    h5path = fullfile(unpackDir, "model.weights.h5");

    if isfile(h5path)
        return;
    end

    if isfolder(unpackDir)
        rmdir(unpackDir, "s");
    end
    mkdir(unpackDir);

    zipPath = fullfile(unpackDir, "model.zip");
    copyfile(modelPath, zipPath);
    unzip(zipPath, unpackDir);
    assert(isfile(h5path), "Keras archive did not contain model.weights.h5: %s", modelPath);
end

function names = i_denseLayerStorageNames(h5path)
    info = h5info(h5path, "/layers");
    names = strings(1, 0);
    for i = 1:numel(info.Groups)
        groupName = string(info.Groups(i).Name);
        shortName = extractAfter(groupName, "/layers/");
        if startsWith(shortName, "dense")
            names(end+1) = shortName; %#ok<AGROW>
        end
    end
    names = sortDenseNames(names);
end

function names = sortDenseNames(names)
    order = zeros(size(names));
    for i = 1:numel(names)
        token = regexp(names(i), "^dense(?:_(\d+))?$", "tokens", "once");
        if isempty(token) || isempty(token{1})
            order(i) = 0;
        else
            order(i) = str2double(token{1});
        end
    end
    [~, idx] = sort(order);
    names = names(idx);
end
