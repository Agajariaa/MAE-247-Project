function B_bidi = utils_partition(B_bid, i, shape)
    if strcmp(shape, 'penta') 
        Ms = [4, 3, 3, 4, 4, 3, 3];
    elseif strcmp(shape, 'hexa') 
        Ms = [5, 5, 5, 8, 7, 7, 8, 5, 5, 5];
    else 
        Ms = []; 
    end
    if i == 1
        idx_start = 1; 
    else
        idx_start = sum(Ms(1:i-1)) + 1; 
    end
    idx_end = idx_start + Ms(i) - 1;
    B_bidi = B_bid(:, idx_start:idx_end);
end