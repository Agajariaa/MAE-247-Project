function render_video(fws, filename, titles, grid_sz, ax_lims, step_size, K_max, edges, m_edges, sc_num)
    fig = figure('Position', [100, 100, 400*grid_sz(2), 400*grid_sz(1)], 'Name', filename, 'Color', 'w');
    v = VideoWriter(filename); v.FrameRate = 10; open(v); 
    
    c_fol = [146, 208, 80]/256; c_ld = [255, 184, 28]/256; c_lst = [1, 0, 0];
    
    h_edges = cell(1, length(fws)); h_foll = cell(1, length(fws)); 
    h_lead = cell(1, length(fws)); h_lost = cell(1, length(fws));
    
    for i = 1:length(fws)
        subplot(grid_sz(1), grid_sz(2), i); hold on;
        fw = fws{i}; traj = fw.config_track;
        xlim(ax_lims(1:2)); ylim(ax_lims(3:4)); title(titles{i}); box on;
        
        for j = 1:fw.N
            col = c_fol; if fw.leader_ID(j), col = c_ld; end
            plot(squeeze(traj(1,j,:)), squeeze(traj(2,j,:)), '-', 'Color', [col, 0.15], 'LineWidth', 0.5); 
        end
        
        h_edges{i} = plot(NaN, NaN, 'k-', 'LineWidth', 1.2);
        h_foll{i}  = plot(NaN, NaN, '.', 'Color', c_fol, 'MarkerSize', 22);
        h_lead{i}  = plot(NaN, NaN, '.', 'Color', c_ld, 'MarkerSize', 22);
        h_lost{i}  = plot(NaN, NaN, '.', 'Color', c_lst, 'MarkerSize', 22);
    end
    
    for t_idx = 1:step_size:K_max
        for i = 1:length(fws)
            fw = fws{i}; traj = fw.config_track; 
            t_traj = min(t_idx, size(traj, 3)); t_sens = min(t_idx, size(fw.sense_mat, 3)); 
            n_xy = squeeze(traj(:, :, t_traj))';
            
            X_edg = []; Y_edg = [];
            for e = 1:m_edges
                u = edges(1, e); v_node = edges(2, e);
                is_ideal = contains(titles{i}, 'Ideal') || contains(titles{i}, '\lambda = 1');
                if is_ideal || fw.sense_mat(u, v_node, t_sens) || fw.sense_mat(v_node, u, t_sens)
                    X_edg = [X_edg, n_xy(u, 1), n_xy(v_node, 1), NaN];
                    Y_edg = [Y_edg, n_xy(u, 2), n_xy(v_node, 2), NaN];
                end
            end
            set(h_edges{i}, 'XData', X_edg, 'YData', Y_edg);

            X_f = []; Y_f = []; X_l = []; Y_l = []; X_x = []; Y_x = [];
            for j = 1:fw.N
                if contains(titles{i}, 'Ideal') || contains(titles{i}, '\lambda = 1')
                    is_lost = false;
                elseif sc_num == 3
                    is_lost = true; 
                else
                    is_lost = (sum(fw.sense_mat(fw.agents{j}.neighbors, j, t_sens)) == 0); 
                end
                
                if is_lost
                    X_x(end+1) = n_xy(j, 1); Y_x(end+1) = n_xy(j, 2);
                elseif fw.leader_ID(j) == 1
                    X_l(end+1) = n_xy(j, 1); Y_l(end+1) = n_xy(j, 2);
                else
                    X_f(end+1) = n_xy(j, 1); Y_f(end+1) = n_xy(j, 2);
                end
            end
            set(h_foll{i}, 'XData', X_f, 'YData', Y_f);
            set(h_lead{i}, 'XData', X_l, 'YData', Y_l);
            set(h_lost{i}, 'XData', X_x, 'YData', Y_x);
        end
        drawnow; 
        writeVideo(v, getframe(fig));
    end
    close(v); close(fig);
end