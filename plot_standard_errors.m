function plot_standard_errors(fws, t, is_log, ax_lims, title_str)
    plot(t, max(fws{1}.tracking_error(), 1e-4), 'b-', 'DisplayName', 'RAL', 'LineWidth', 1.2); 
    plot(t, max(fws{2}.tracking_error(), 1e-4), 'k--', 'DisplayName', 'RKF', 'LineWidth', 2);
    plot(t, max(fws{3}.tracking_error(), 1e-4), 'r-', 'DisplayName', 'GA-RKF', 'LineWidth', 2);
    plot(t, max(fws{4}.tracking_error(), 1e-4), 'g:', 'DisplayName', 'No Loss (Ideal)', 'LineWidth', 2);
    grid on; legend('show', 'Location', 'northwest'); hold off;
    if is_log, set(gca, 'YScale', 'log'); ylabel('Avg Tracking Error (Log Scale)'); else, ylabel('Avg Tracking Error (Linear)'); end
    xlim([0, max(t)]); ylim(ax_lims); xlabel('Time (s)'); title(title_str);
end

%[appendix]{"version":"1.0"}
%---
