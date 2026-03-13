classdef Framework < handle
    properties
        dt, K_max, static, shape
        B_und, B_nom, N, M_und, M_nom, L, leader_ID
        D, P, Z_init, Z_target, Theta_target
        config_track, U_last, R, obs_noise, sense_mat, agents
    end
    
    methods
        function obj = Framework(sim_params)
            obj.dt = sim_params.dt;
            obj.K_max = round(sim_params.t_max / obj.dt);
            obj.static = sim_params.static;
            obj.shape = sim_params.shape;
            
            [obj.B_und, obj.B_nom, obj.N, obj.M_und, obj.M_nom, obj.L, obj.leader_ID] = obj.graph_setup(obj.shape);
            [obj.N, obj.D, obj.P, obj.Z_init, obj.Z_target, obj.Theta_target] = obj.config_setup(obj.shape, obj.K_max, obj.static);
            
            obj.config_track = zeros(obj.D, obj.N, obj.K_max + 1);
            obj.config_track(:, :, 1) = obj.Z_init;
            obj.U_last = zeros(obj.D, obj.N);
            
            obj.R = sim_params.sigma_v^2 * [1, 0.3; 0.3, 1];
            total_samples = obj.K_max * obj.N * obj.N;
            noise = mvnrnd(zeros(1, obj.D), obj.R, total_samples);
            obj.obs_noise = reshape(noise, [obj.K_max, obj.N, obj.N, obj.D]);
            
            obj.sense_mat = sim_params.sense_mat;
            
            obj.agents = cell(1, obj.N);
            for i = 1:obj.N
                obj.agents{i} = Agent(i, obj.sense_mat, obj.P, obj.Z_init, obj.dt, ...
                    obj.B_nom, obj.B_und, obj.L, obj.D, obj.leader_ID(i), obj.Z_target, obj.K_max, obj.static);
            end
        end
        
        function [B_und, B_bid, N, M_und, M_bid, L, leader_ID] = graph_setup(obj, shape)
            [B_und, B_bid] = graph_params(shape);
            [N, M_und] = size(B_und);
            [~, M_bid] = size(B_bid);
            
            if strcmp(shape, 'penta')
                leader_ID = [1, 1, 1, 0, 0, 0, 0];
                data = load('data/stress_penta.mat');
                L = data.L;
            elseif strcmp(shape, 'hexa')
                leader_ID = [0, 0, 0, 1, 1, 0, 1, 0, 0, 0];
                data = load('data/stress_hexa.mat');
                L = data.L;
            else
                leader_ID = []; L = [];
            end
        end
        
        function [N, D, P, Z_init, Z_target, Theta_target] = config_setup(obj, shape, K_max, static)
            [N, D, P] = nominal_config(shape);
            Z_target = zeros(D, N, K_max);
            
            if static
                Theta_target_base = eye(2);
                target_config = Theta_target_base * P;
                Z_target = repmat(target_config, [1, 1, K_max]);
                Theta_target = repmat(Theta_target_base, [1, 1, K_max]);
            else
                dataA = load('data/true_param.mat');
                Theta_target_raw = dataA.A; 
                step = round(obj.dt / 0.001);
                Theta_target = Theta_target_raw(:, :, 1:step:end);
                Theta_target = Theta_target(:, :, 1:K_max);
                
                datat = load('data/trans.mat');
                trans_target_raw = datat.t; 
                trans_target_step = trans_target_raw(:, 1:step:end);
                trans_target_step = trans_target_step(:, 1:K_max);
                
                for k = 1:K_max
                    Z_target(:, :, k) = Theta_target(:, :, k) * P + repmat(trans_target_step(:, k), [1, N]);
                end
            end
            
            % Set initial positions to perfectly match the first frame of the target trajectory
            Z_init = Z_target(:, :, 1);
        end
        
        function error_out = tracking_error(obj)
            error_diff = obj.config_track(:, :, 1:end-1) - obj.Z_target;
            error_out = (1 / obj.N) * squeeze(sum(error_diff.^2, [1, 2]));
        end
        
        function error_out = tracking_error_ind(obj)
            error_diff = obj.config_track(:, :, 1:end-1) - obj.Z_target;
            error_out = squeeze(sum(error_diff.^2, 1));
        end
        
        function run(obj)
            for k = 1:obj.K_max
                for i = 1:obj.N
                    Zij = obj.config_track(:, i, k) - obj.config_track(:, :, k);
                    Z = obj.config_track(:, :, k);
                    
                    Yij = Zij + squeeze(obj.obs_noise(k, i, :, :))';
                    Yij = Yij .* repmat(obj.sense_mat(:, i, k)', 2, 1);
                    
                    [obj.config_track(:, i, k + 1), obj.U_last(:, i)] = obj.agents{i}.step(Z, obj.U_last, Yij, k);
                end
            end
        end
    end
end