function deflection = computeRebarDeflection(diameter_mm, length_m, gripPoint)
%COMPUTEREBARDEFLECTION  Compute retreat lift deflection (meters).
%   deflection = computeRebarDeflection(diameter_mm, length_m, gripPoint)
%   gripPoint: "center"/"centre" or "end"/"one-end"

    % ---- normalize diameter_mm ----
    if isstring(diameter_mm) || ischar(diameter_mm)
        diameter_mm = str2double(diameter_mm);
    elseif iscell(diameter_mm)
        diameter_mm = diameter_mm{1};
    end
    validateattributes(diameter_mm, {'numeric'}, {'real','finite','positive'});
    diameter_mm = double(diameter_mm);
    if ~isscalar(diameter_mm), diameter_mm = diameter_mm(1); end

    % ---- normalize length_m ----
    if isstring(length_m) || ischar(length_m)
        length_m = str2double(length_m);
    elseif iscell(length_m)
        length_m = length_m{1};
    end
    validateattributes(length_m, {'numeric'}, {'real','finite','positive'});
    length_m = double(length_m);
    if ~isscalar(length_m), length_m = length_m(1); end

    % ---- normalize gripPoint ----
    if isstring(gripPoint), gripPoint = char(gripPoint); end
    gripPoint = lower(strtrim(gripPoint));

    switch gripPoint
        case {'center','centre'}
            effLen = length_m / 2;
        case {'end','one-end','oneend','one_end','tip'}
            effLen = length_m;
        otherwise
            warning('Unknown gripPoint "%s". Defaulting to "end".', gripPoint);
            effLen = length_m;
    end

    % ---- call the predictor (expects mm + m) ----
    deflection = deflection_predictor(diameter_mm, effLen);  % metres

    % Optional bounds:
    % deflection = max(0.03, min(0.40, deflection));  % clamp 3–40 cm
end
