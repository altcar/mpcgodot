function mpc_main()
    %% MATLAB NMPC Lane Following + Obstacle Avoidance
    
    godotIP = "127.0.0.1";
    godotPort = 4246;
    matlabPort = 4247;
    
    expectedDoubles = 698;
    expectedBytes = expectedDoubles * 8;
    
    delete(findall(0, "Type", "udpport"));
    u_port = udpport("LocalPort", matlabPort);
    
    %% Visualization
    figure('Name', 'NMPC Lane + Obstacle Avoidance', ...
           'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    
    subplot(2,2,[1 3]);
    theta = deg2rad(0:359);
    hPolar = polarplot(theta, zeros(1, 360), 'LineWidth', 1.5);
    title('LiDAR 360°'); rlim([0 50]); grid on;
    
    subplot(2,2,2);
    hold on; colors = jet(5); hLanes = gobjects(5,1);
    for i=1:5
        hLanes(i) = plot(linspace(-4,4,64), zeros(1,64), ...
            'LineWidth',1.5,'Color',colors(i,:));
    end
    title('Lane Horizons'); ylim([-0.1 1.1]); grid on;
    
    subplot(2,2,4);
    hCTE = plot(0:5:20, zeros(1,5), 'r-o','LineWidth',2);
    yline(0,'k--'); ylim([-3 3]); grid on;
    title('CTE Horizon');
    
    fprintf("NMPC Lane + Obstacle Avoidance Running...\n");
    
    try
        last_steer = 0.0;
        speed_cmd = 15.0;
        
        while true
            
            % Send control
            write(u_port, [last_steer, speed_cmd], "double", godotIP, godotPort);
            
            % Read sensors
            [has_data, state, laser, lane_horizons, cte_horizon] = ...
                read_udp_sensors(u_port, expectedBytes, expectedDoubles);
            
            if has_data
                
                %% Snap to nearest lane
                current_lane_offset = round(cte_horizon(1) / 3.5) * 3.5;
                snapped_horizon = cte_horizon - current_lane_offset;
                
                %% NMPC with obstacle avoidance
                last_steer = lane_following_nmpc( ...
                    state, snapped_horizon, speed_cmd, last_steer, laser);
                
                %% Visualization
                set(hPolar, 'YData', laser);
                for i=1:5
                    set(hLanes(i), 'YData', lane_horizons(i,:));
                end
                set(hCTE, 'YData', snapped_horizon);
                drawnow limitrate;
            end
            
            pause(0.005);
        end
        
    catch ME
        fprintf("\n[STOPPED]: %s\n", ME.message);
        clear u_port;
    end
end

%% ================= SENSOR READER =================
function [has_data, state, laser, lane_horizons, cte_horizon] = ...
    read_udp_sensors(u, expectedBytes, expectedDoubles)

    has_data = false;
    state = []; laser = []; lane_horizons = []; cte_horizon = [];
    
    while u.NumBytesAvailable >= expectedBytes
        all_data = read(u, expectedDoubles, "double");
        has_data = true;
    end
    
    if has_data && ~any(isnan(all_data))
        state         = all_data(1:13);
        laser         = all_data(14:373);
        lane_horizons = reshape(all_data(374:693), [64, 5])';
        cte_horizon   = all_data(694:698);
    else
        has_data = false;
    end
end

%% ================= NMPC CONTROLLER =================
function opt_steer = lane_following_nmpc(state, cte_horizon, v, last_steer, laser)

    N = 5;
    dt = 5.0 / v;
    
    obj_fun = @(w) compute_nmpc_cost(w, cte_horizon, dt, v, last_steer, laser);
    
    lb = -1.5 * ones(N,1);
    ub =  1.5 * ones(N,1);
    
    w0 = last_steer * ones(N,1);
    
    options = optimoptions('fmincon','Display','off','Algorithm','sqp');
    w_opt = fmincon(obj_fun, w0, [], [], [], [], lb, ub, [], options);
    
    opt_steer = w_opt(1);
end

%% ================= COST FUNCTION =================
function cost = compute_nmpc_cost(w, cte_ref, dt, v, last_steer, laser)

    N = length(w);
    cost = 0;
    
    % State
    x = 0;
    y = 0;
    theta = 0;
    
    % Weights
    W_cte = 100;
    W_u   = 5;
    W_du  = 50;
    W_obs = 3000;
    
    safe_dist = 2.5;
    
    prev_w = last_steer;
    
    %% Convert LiDAR to Cartesian
    angles = deg2rad(0:359);
    obs_x = laser .* cos(angles);
    obs_y = laser .* sin(angles);
    
    % Only nearby + forward obstacles
    valid = (laser < 20) & (abs(angles) < pi/2);
    obs_x = obs_x(valid);
    obs_y = obs_y(valid);
    
    for k = 1:N
        
        %% Predict motion
        theta = theta + w(k)*dt;
        x = x + v*cos(theta)*dt;
        y = y + v*sin(theta)*dt;
        
        %% Lane tracking error
        err = y - cte_ref(k);
        
        cost = cost + ...
            W_cte*(err^2) + ...
            W_u*(w(k)^2) + ...
            W_du*((w(k)-prev_w)^2);
        
        prev_w = w(k);
        
        %% Obstacle avoidance cost
        if ~isempty(obs_x)
            dists = sqrt((obs_x - x).^2 + (obs_y - y).^2);
            min_dist = min(dists);
            
            if min_dist < safe_dist
                cost = cost + W_obs*(safe_dist - min_dist)^2;
            end
        end
        
    end
end