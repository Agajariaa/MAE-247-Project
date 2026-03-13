classdef Framework_FLT < Framework
    properties
        Theta_est_track, Theta_est_now, Theta_conRAL_track
        use_RAL, use_conRAL, use_GARKF, use_RKFIO
        Phi_track, psi_track, ci_track, bi_track
        sigma_w, state, cov, cov_track
    end
    methods
        function obj = Framework_FLT(sim_params)
            obj@Framework(sim_params); 
            obj.Theta_est_track = zeros(obj.D, obj.D, obj.N, obj.K_max);
            obj.Theta_est_now = zeros(obj.D, obj.D, obj.N);
            obj.Theta_conRAL_track = zeros(obj.D, obj.D, obj.N, obj.K_max + 1);

            obj.agents = cell(1, obj.N);
            for i = 1:obj.N
                obj.agents{i} = Agent_FLT(i, obj.sense_mat, obj.P, obj.Z_init, obj.dt, ...
                    obj.B_nom, obj.B_und, obj.L, obj.D, obj.leader_ID(i), obj.Z_target, obj.K_max, obj.static);
            end

            obj.use_RAL = sim_params.RAL; obj.use_conRAL = sim_params.conRAL;
            obj.use_GARKF = sim_params.GARKF; obj.use_RKFIO = sim_params.RKFIO;

            obj.Phi_track = cell(1, obj.N);
            for i = 1:obj.N, obj.Phi_track{i} = zeros(obj.D, length(obj.agents{i}.neighbors), obj.K_max); end
            obj.psi_track = zeros(obj.N, obj.K_max); obj.ci_track = zeros(obj.N, obj.K_max); obj.bi_track = zeros(obj.N, obj.K_max);

            obj.sigma_w = 0.01; obj.state = zeros(obj.N, obj.N, 6);
            obj.cov = permute(repmat(2 * eye(6), [1, 1, obj.N, obj.N]), [3, 4, 1, 2]);
            obj.cov_track = zeros(6, 6, obj.K_max);
        end

        function est_Theta(obj, k)
            for i = 1:obj.N
                Bi = utils_partition(obj.B_nom, i, obj.shape);
                sel_seq = obj.sense_mat(obj.agents{i}.neighbors, i, k);
                valid_idx = find(sel_seq);
                
                Zij = obj.config_track(:, i, k) - obj.config_track(:, :, k);
                Yij = Zij + squeeze(obj.obs_noise(k, i, :, :))';
                Yij = Yij(:, obj.agents{i}.neighbors);
                Hi = obj.P * Bi(:, valid_idx);

                if rank(Hi * Hi') == obj.D
                    Phi = inv(Hi * Hi') * Hi; Theta = Yij(:, valid_idx) * Phi';
                    obj.Theta_est_track(:, :, i, k) = Theta; obj.Theta_est_now(:, :, i) = Theta;
                    obj.agents{i}.store_Phi(Phi);
                else
                    if k == 1, obj.Theta_est_now(:, :, i) = zeros(2,2);
                    else, obj.Theta_est_now(:, :, i) = obj.Theta_conRAL_track(:, :, i, k - 1); end
                end
            end
        end

        function Zij_est = RAL(obj, i, k, Yij)
            if obj.use_conRAL, Theta = obj.Theta_conRAL_track(:, :, i, k); else, Theta = obj.Theta_est_track(:, :, i, k); end
            Zij_est = Theta * (obj.P(:, i) - obj.P); Zij_est(:, find(obj.sense_mat(:, i, k))) = Yij(:, find(obj.sense_mat(:, i, k)));
        end

        function CI(obj, k)
            for i = 1:obj.N
                Nik = length(obj.agents{i}.neighbors); psii = 0;
                for j_idx = 1:Nik
                    j = obj.agents{i}.neighbors(j_idx); psii = psii + norm(obj.Theta_est_track(:, :, i, k) - obj.Theta_est_track(:, :, j, k))^2;
                end
                obj.bi_track(i, k) = 0; obj.ci_track(i, k) = 0; obj.psi_track(i, k) = (1 / Nik) * psii;
            end
        end

        function ConRAL(obj, k)
            if k == 1, obj.Theta_conRAL_track(:, :, :, 1) = obj.Theta_est_track(:, :, :, 1); end
            for i = 1:obj.N
                idx_sel_seq = find(obj.sense_mat(:, i, k)); neib_sum = zeros(obj.D, obj.D); neib_sum1 = zeros(obj.D, obj.D);
                for j_idx = 1:length(obj.agents{i}.neighbors)
                    j = obj.agents{i}.neighbors(j_idx);
                    if ismember(j, idx_sel_seq)
                        neib_sum = neib_sum + obj.Theta_conRAL_track(:, :, j, k) - obj.Theta_conRAL_track(:, :, i, k);
                        if all(obj.Theta_est_track(:, :, j, k) > 1e-5, 'all'), neib_sum1 = neib_sum1 + obj.Theta_est_track(:, :, j, k) - obj.Theta_conRAL_track(:, :, i, k); end
                    end
                end
                if all(obj.Theta_est_track(:, :, i, k) > 1e-5, 'all'), neib_sum1 = neib_sum1 + obj.Theta_est_track(:, :, i, k) - obj.Theta_conRAL_track(:, :, i, k); end
                obj.Theta_conRAL_track(:, :, i, k + 1) = obj.Theta_conRAL_track(:, :, i, k) + 0.08 * (neib_sum + neib_sum1);
            end
        end

        function [cstate, ccov, est_val] = KF(obj, state_in, cov_in, yij, R_in, is_obs)
            Q = (obj.sigma_w ^ 2) * [0.25*obj.dt^4, 0.5*obj.dt^3, 0.5*obj.dt^2, 0, 0, 0; 0.5*obj.dt^3, obj.dt^2, obj.dt, 0, 0, 0; 0.5*obj.dt^2, obj.dt, 1, 0, 0, 0; 0, 0, 0, 0.25*obj.dt^4, 0.5*obj.dt^3, 0.5*obj.dt^2; 0, 0, 0, 0.5*obj.dt^3, obj.dt^2, obj.dt; 0, 0, 0, 0.5*obj.dt^2, obj.dt, 1];
            G = [1, 0, 0, 0, 0, 0; 0, 0, 0, 1, 0, 0];
            F = [1, obj.dt, 0.5*obj.dt^2, 0, 0, 0; 0, 1, obj.dt, 0, 0, 0; 0, 0, 1, 0, 0, 0; 0, 0, 0, 1, obj.dt, 0.5*obj.dt^2; 0, 0, 0, 0, 1, obj.dt; 0, 0, 0, 0, 0, 1];
            state_vec = squeeze(state_in); if isrow(state_vec), state_vec = state_vec'; end
            pstate = F * state_vec; pcov = F * squeeze(cov_in) * F' + Q;
            if is_obs
                K = pcov * G' * inv(R_in + G * pcov * G'); cstate = pstate + K * (yij - G * pstate); ccov = (eye(6) - K * G) * pcov;
            else
                cstate = pstate; ccov = pcov;
            end
            est_val = G * cstate;
        end

        function Zij_est = GARKF(obj, i, k, Yij)
            idx_sel_seq = find(obj.sense_mat(:, i, k)); Zij_est = zeros(obj.D, obj.N);
            for j_idx = 1:length(obj.agents{i}.neighbors)
                j = obj.agents{i}.neighbors(j_idx);
                if ismember(j, idx_sel_seq), R_val = obj.R; else, R_val = obj.R + obj.psi_track(i, k) * eye(obj.D); if obj.use_conRAL, R_val = (1/obj.N)*R_val; end; end
                [cstate, ccov, Zij_est(:, j)] = obj.KF(obj.state(i, j, :), obj.cov(i, j, :, :), Yij(:, j), R_val, true);
                obj.state(i, j, :) = cstate; obj.cov(i, j, :, :) = ccov;
                if i == 1 && j == 2, obj.cov_track(:, :, k) = ccov; end
            end
        end

        function Zij_est = RKFIO(obj, i, k, Yij)
            idx_sel_seq = find(obj.sense_mat(:, i, k)); Zij_est = zeros(obj.D, obj.N);
            for j_idx = 1:length(obj.agents{i}.neighbors)
                j = obj.agents{i}.neighbors(j_idx);
                [cstate, ccov, Zij_est(:, j)] = obj.KF(obj.state(i, j, :), obj.cov(i, j, :, :), Yij(:, j), obj.R, ismember(j, idx_sel_seq));
                obj.state(i, j, :) = cstate; obj.cov(i, j, :, :) = ccov;
                if i == 1 && j == 2, obj.cov_track(:, :, k) = ccov; end
            end
        end

        function run(obj)
            for k = 1:obj.K_max
                obj.est_Theta(k); obj.CI(k); if obj.use_conRAL, obj.ConRAL(k); end
                for i = 1:obj.N
                    Zij = obj.config_track(:, i, k) - obj.config_track(:, :, k); Z = obj.config_track(:, :, k);
                    Yij = (Zij + squeeze(obj.obs_noise(k, i, :, :))') .* repmat(obj.sense_mat(:, i, k)', 2, 1);
                    if obj.use_RAL, Zij_est = obj.RAL(i, k, Yij); if obj.use_GARKF, Zij_est = obj.GARKF(i, k, Zij_est); end
                    elseif obj.use_RKFIO, Zij_est = obj.RKFIO(i, k, Yij); else, Zij_est = Yij; end
                    [obj.config_track(:, i, k + 1), obj.U_last(:, i)] = obj.agents{i}.step(Z, obj.U_last, Zij_est, k);
                end
            end
        end
    end
end