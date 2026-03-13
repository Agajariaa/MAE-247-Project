function sense_mat = swi_top_sensemat(shape, dt, t_max)
    if strcmp(shape, 'penta')
        sec1 = 15; sense_mat_mode1 = repmat(ones(7, 7), [1, 1, round(sec1 / dt)]);
        sec2 = 15; sense_mat_mode2 = repmat([0,1,1,1,1,0,0; 1,0,0,1,0,0,1; 1,0,0,0,1,1,0; 0,1,0,0,1,1,0; 1,0,1,0,0,0,1; 0,0,0,1,0,0,1; 0,0,0,0,1,1,0], [1, 1, round(sec2 / dt)]);
        sec3 = 15; sense_mat_mode3 = repmat([0,1,1,1,1,0,0; 1,0,0,1,0,0,0; 1,0,0,0,1,0,0; 1,1,0,0,1,1,0; 1,0,1,1,0,0,1; 0,0,0,1,0,0,1; 0,0,0,0,1,1,0], [1, 1, round(sec3 / dt)]);
        sec4 = 15; sense_mat_mode4 = repmat([0,1,1,1,1,0,1; 1,0,0,1,0,0,1; 1,0,0,0,1,1,1; 1,1,0,0,1,1,1; 1,0,1,1,0,0,1; 0,0,1,1,0,0,1; 0,0,0,0,0,0,1], [1, 1, round(sec4 / dt)]);
        sense_mat = cat(3, sense_mat_mode1, sense_mat_mode2, sense_mat_mode3, sense_mat_mode4);
    elseif strcmp(shape, 'hexa')
        sec1 = 10; sense_mat_mode1 = repmat(ones(10, 10), [1, 1, round(sec1 / dt)]);
        sec2 = 25; sense_mat_mode2 = repmat([0,1,1,1,1,1,0,0,0,1; 1,0,1,1,1,0,1,0,0,1; 1,1,0,1,0,1,1,0,0,1; 1,1,1,0,1,1,1,1,1,1; 1,1,0,1,0,1,1,1,0,1; 1,0,1,1,1,0,1,0,1,1; 0,1,1,1,1,1,0,1,1,1; 0,0,0,1,1,0,1,0,1,1; 0,0,0,1,0,1,1,1,0,1; 0,0,0,0,0,0,0,0,0,1], [1, 1, round(sec2 / dt)]);
        sec3 = 25; sense_mat_mode3 = repmat([0,1,1,1,1,1,0,0,0,1; 1,0,1,1,1,0,1,0,0,1; 0,0,1,0,0,0,0,0,0,1; 1,1,1,0,1,1,1,1,1,1; 1,1,1,1,0,1,1,1,0,1; 1,0,1,1,1,1,1,0,1,1; 0,1,1,1,1,1,0,1,1,1; 0,0,1,1,1,0,1,0,1,1; 0,0,1,1,0,1,1,1,0,1; 0,0,1,0,0,0,0,0,0,1], [1, 1, round(sec3 / dt)]);
        sense_mat = cat(3, sense_mat_mode1, sense_mat_mode2, sense_mat_mode3);
    else
        sense_mat = [];
    end
end