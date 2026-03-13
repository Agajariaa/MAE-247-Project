classdef Agent_FLT < Agent
    properties
        Phi_now
    end
    methods
        function obj = Agent_FLT(ID, sense_mat, P, Z, dt, B_nom, B_und, L, D, is_leader, Z_target, K_max, is_static)
            obj@Agent(ID, sense_mat, P, Z, dt, B_nom, B_und, L, D, is_leader, Z_target, K_max, is_static);
        end
        function store_Phi(obj, Phi)
            obj.Phi_now = Phi;
        end
        function valid_neighbor = valid_neighbors(obj, k)
            valid_idx = find(obj.sense_mat(:, obj.ID, k));
            valid_neighbor = intersect(obj.neighbors, valid_idx);
        end
        function [z_out, u_out] = step(obj, Y, U, Yij, k)
            if obj.is_leader == 0
                u = zeros(obj.D, 1);
                if obj.static == true
                    for idx = 1:length(obj.neighbors)
                        j = obj.neighbors(idx); u = u + 50 * obj.L(obj.ID, j) * Yij(:, j);
                    end
                else
                    gamma = 0;
                    for idx = 1:length(obj.neighbors)
                        j = obj.neighbors(idx); gamma = gamma + obj.L(obj.ID, j);
                        u = u + obj.L(obj.ID, j) * (Yij(:, j) - U(:, j));
                    end
                    u = -(1 / gamma) * u;
                end
            else
                if obj.static == true
                    u = -50 * (Y(:, obj.ID) - obj.Z_target(:, obj.ID, k));
                else
                    if k == 1, dz_ref = zeros(obj.D, 1);
                    else, dz_ref = (obj.Z_target(:, obj.ID, k) - obj.Z_target(:, obj.ID, k - 1)) / obj.dt; end
                    u = -10 * (Y(:, obj.ID) - obj.Z_target(:, obj.ID, k)) + dz_ref;
                end
            end
            
            % --- THE VIOLATION: NONLINEAR ACTUATOR SATURATION ---
            % This caps the drone's speed, preventing the linear controller 
            % from instantly rescuing a bad RAL geometry estimate.
            max_v = 1.5; % Maximum speed in m/s
            speed = norm(u);
            if speed > max_v
                u = (u / speed) * max_v;
            end
            
            obj.z = obj.z + obj.dt * u;
            z_out = obj.z; u_out = u;
        end
    end
end