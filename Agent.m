classdef Agent < handle
    properties
        dt, sense_mat, B_nom, B_und, N, M, L
        ID, is_leader, D, P, z, neighbors
        Z_target, K_max, static
    end
    
    methods
        function obj = Agent(ID, sense_mat, P, Z, dt, B_nom, B_und, L, D, is_leader, Z_target, K_max, is_static)
            obj.dt = dt;
            obj.sense_mat = sense_mat;
            obj.B_nom = B_nom;
            obj.B_und = B_und;
            [obj.N, obj.M] = size(obj.B_nom);
            obj.L = L;
            obj.ID = ID;
            obj.is_leader = is_leader;
            obj.D = D;
            obj.P = P;
            obj.z = Z(:, obj.ID);
            obj.neighbors = obj.get_neighbors();
            obj.Z_target = Z_target;
            obj.K_max = K_max;
            obj.static = is_static;
        end
        
        function neighbor_ID = get_neighbors(obj)
            edges = find(obj.B_und(obj.ID, :));
            [rows, ~] = find(obj.B_und(:, edges));
            neighbor_ID = unique(rows);
            neighbor_ID(neighbor_ID == obj.ID) = []; % remove self
            neighbor_ID = reshape(neighbor_ID, 1, []); % ensure row vector
        end
        
        function valid_neighbor = valid_neighbors(obj, k)
            valid_idx = find(obj.sense_mat(:, obj.ID, k));
            valid_neighbor = intersect(obj.neighbors, valid_idx);
        end
        
        function [z_out, u_out] = step(obj, Y, U, Yij, k)
            if obj.is_leader == 0
                u = zeros(obj.D, 1);
                if obj.static
                    for idx = 1:length(obj.neighbors)
                        j = obj.neighbors(idx);
                        u = u + 50 * obj.L(obj.ID, j) * Yij(:, j);
                    end
                else
                    gamma = 0;
                    for idx = 1:length(obj.neighbors)
                        j = obj.neighbors(idx);
                        gamma = gamma + obj.L(obj.ID, j);
                        u = u + obj.L(obj.ID, j) * (Yij(:, j) - U(:, j));
                    end
                    u = -(1 / gamma) * u;
                end
            else
                if obj.static
                    u = -50 * (Y(:, obj.ID) - obj.Z_target(:, obj.ID, k));
                else
                    if k == 1
                        dz_ref = zeros(obj.D, 1);
                    else
                        dz_ref = (obj.Z_target(:, obj.ID, k) - obj.Z_target(:, obj.ID, k - 1)) / obj.dt;
                    end
                    u = -10 * (Y(:, obj.ID) - obj.Z_target(:, obj.ID, k)) + dz_ref;
                end
            end
            obj.z = obj.z + obj.dt * u;
            z_out = obj.z;
            u_out = u;
        end
    end
end