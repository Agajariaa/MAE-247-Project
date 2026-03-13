function fws = run_standard_fws(sp, Z_targ, K_max)
    flags = [1 0 0 0; 0 0 0 1; 1 0 1 0; 0 0 0 0]; fws = cell(1, 4);
    for i = 1:4
        sp.RAL = flags(i,1); sp.conRAL = flags(i,2); sp.GARKF = flags(i,3); sp.RKFIO = flags(i,4);
        if i == 4, sp.Bern_prob = 1; sp.sense_mat = double(rand(10, 10, K_max) <= 1); end
        fws{i} = Framework_FLT(sp);
        if ~isempty(Z_targ), fws{i}.Z_target = Z_targ; for j = 1:fws{i}.N, fws{i}.agents{j}.Z_target = Z_targ; end; end
        fws{i}.run();
    end
end

%[appendix]{"version":"1.0"}
%---
