function fw = run_single_fw(sp, s_mat, Z_targ, is_ral, is_conral, is_rkf, is_garkf)
    sp.sense_mat = s_mat; sp.RAL = is_ral; sp.conRAL = is_conral; sp.GARKF = is_garkf; sp.RKFIO = is_rkf;
    fw = Framework_FLT(sp); 
    if ~isempty(Z_targ), fw.Z_target = Z_targ; for j = 1:fw.N, fw.agents{j}.Z_target = Z_targ; end; end
    fw.run(); 
end

%[appendix]{"version":"1.0"}
%---
