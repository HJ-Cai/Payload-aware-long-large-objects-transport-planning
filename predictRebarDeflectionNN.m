function y = predictRebarDeflectionNN(diameter_mm, length_m)
%PREDICTREBARDEFLECTIONNN  Predict rebar deflection (meters) using ONNX model.
%   y = predictRebarDeflectionNN(diameter_mm, length_m)
%   Inputs:
%       diameter_mm : rebar diameter in millimetres (scalar > 0)
%       length_m    : rebar length in metres (scalar > 0)
%   Output:
%       y           : predicted deflection in metres (scalar)
%
%   Requirements in current folder (or selectable on first call):
%       - deflection_nn_seed11.onnx
%       - defl_scaler.json  with fields: mean (1x2), scale (1x2)
%
%   Notes:
%       • Uses persistent net/scaler so repeated calls are fast
%       • Feature order must be [diameter_m, length_m]
%
%   Example:
%       y = predictRebarDeflectionNN(16, 3.0);

    arguments
        diameter_mm (1,1) double {mustBeFinite, mustBePositive}
        length_m    (1,1) double {mustBeFinite, mustBePositive}
    end

    persistent NET MU SIG
    if isempty(NET) || isempty(MU) || isempty(SIG)
        [NET, MU, SIG] = i_loadModelAndScaler();
    end

    X  = [diameter_mm/1000.0, length_m];  % metres
    Xs = single((X - MU) ./ SIG);         % StandardScaler

    % Forward pass (supports DAG/Series/dlnetwork)
    if isa(NET,'dlnetwork')
        dlX = dlarray(Xs,'BC');
        ydl = forward(NET, dlX);
        y   = double(extractdata(ydl(1)));
    else
        yhat = predict(NET, Xs);
        y    = double(yhat(1));
    end
end

% --- helpers ---------------------------------------------------------------
function [net, mu, sigma] = i_loadModelAndScaler()
    % auto-locate files if default names not found
    onnxPath   = i_ensureFile("deflection_nn_seed11.onnx", "*.onnx", "Select ONNX model");
    scalerJSON = i_ensureFile("defl_scaler.json", "*.json", "Select scaler JSON");

    % Import ONNX (prefer new API; fall back to old; then layers->assemble)
    net = i_importONNX(onnxPath);

    % Load StandardScaler params from JSON
    S = jsondecode(fileread(scalerJSON));
    mu    = double(S.mean(:)).';
    sigma = double(S.scale(:)).';
    assert(numel(mu)==2 && numel(sigma)==2, 'Scaler JSON must have exactly 2 features.');
end

function net = i_importONNX(onnxPath)
    if exist('importNetworkFromONNX','file')
        try
            net = importNetworkFromONNX(onnxPath, ...
                  "InputDataFormats","BC", "OutputDataFormats","BC");
            return;
        catch
            % continue
        end
    end
    if exist('importONNXNetwork','file')
        try
            net = importONNXNetwork(onnxPath, ...
                  "InputDataFormats","BC", "OutputDataFormats","BC");
            return;
        catch
            % continue
        end
    end
    if exist('importONNXLayers','file')
        lgraph = importONNXLayers(onnxPath, ...
                 "InputDataFormats","BC", "OutputDataFormats","BC", ...
                 "ImportWeights", true);
        net = assembleNetwork(lgraph);  % may error if placeholders exist
        return;
    end
    error('No suitable ONNX importer found. Install the ONNX converter support package.');
end

function p = i_ensureFile(defaultName, pattern, promptTitle)
    if isfile(defaultName), p = defaultName; return; end
    d = dir(pattern);
    if ~isempty(d), p = d(1).name; return; end
    try d = dir("**/" + pattern); catch, d = []; end
    if ~isempty(d), p = fullfile(d(1).folder, d(1).name); return; end
    [fn,fp] = uigetfile(pattern, promptTitle);
    if isequal(fn,0), error('No file selected for %s', promptTitle); end
    p = fullfile(fp,fn);
end

