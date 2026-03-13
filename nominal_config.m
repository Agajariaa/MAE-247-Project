function [N, D, P] = nominal_config(shape)
    if strcmp(shape, 'penta')
        N = 7; D = 2; P = [2, 1, 1, 0, 0, -1, -1; 0, 1, -1, 1, -1, 1, -1];
    elseif strcmp(shape, 'hexa')
        N = 10; D = 2; P_temp = [3, 0; 2, 2; 2, -2; 1, 0; 0, 2; 0, -2; -1, 0; -2, 2; -2, -2; -3, 0]; P = P_temp'; 
    else
        N = 0; D = 0; P = [];
    end
end