function mpc_main()
    %% MATLAB NMPC Lane Following Controller
    % Packet Layout: 13 (State) + 360 (Lidar) + 320 (Lane Horizons) + 5 (CTE Horizon) = 698 doubles
    % Byte Size: 698 * 8 = 5584 bytes
    
    % 1. INITIALIZATION & CLEANUP
    godotIP = "127.0.0.1";
    godotPort = 4246;
    matlabPort = 4247;
    expectedDoubles = 698; % 13 State + 360 Lidar + 320 Lane (5x64) + 5 CTE (Horizon)
    expectedBytes = expectedDoubles * 8;
    
    delete(findall(0, "Type", "udpport"));
    u_port = udpport("LocalPort", matlabPort);
    
    % Setup Visualization
    figure('Name', 'NMPC Lane Following Monitor', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    
    % Subplot 1: Polar LiDAR
    subplot(2,2,[1 3]);
    theta = deg2rad(0:359);
    hPolar = polarplot(theta, zeros(1, 360), 'LineWidth', 1.5, 'Color', 'b');
    title('LiDAR 360° Scan');
    rlim([0 50]); grid on;
    
    % Subplot 2: Multi-Horizon Lane Intensity
    subplot(2,2,2);
    hold on; colors = jet(5); hLanes = gobjects(5,1);
    for i=1:5
        hLanes(i) = plot(linspace(-4, 4, 64), zeros(1, 64), 'LineWidth', 1.5, 'Color', colors(i,:), ...
            'DisplayName', sprintf('H=%dm', (i-1)*5));
    end
    title('Ground Lane Patterns (Multi-Horizon)');
    xlabel('Lateral Offset (m)'); ylabel('Intensity');
    ylim([-0.1 1.1]); legend('Location', 'best'); grid on;
    
    % Subplot 3: NMPC CTE Horizon
    subplot(2,2,4);
    hCTE = plot(0:5:20, zeros(1, 5), 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
    yline(0, 'k--', 'Center Line', 'LineWidth', 1.5);
    title('Look-ahead CTE & NMPC Tracking');
    xlabel('Look-ahead Distance (m)'); ylabel('CTE (m)');
    ylim([-3 3]); grid on;
    
    fprintf("====================================================\n");
    fprintf("MATLAB NMPC Lane Following Active\n");
    fprintf("====================================================\n\n");
    
    try
        last_steer = 0.0;
        speed_cmd = 15.0;
        
        while true
            % Send Control to Godot
            write(u_port, [last_steer, speed_cmd], "double", godotIP, godotPort);
            
            % Read Sensor Data
            [has_data, state, laser, lane_horizons, cte_horizon] = read_udp_sensors(u_port, expectedBytes, expectedDoubles);
            
            if has_data
                % --- SNAP TO CURRENT LANE ---
                % Prevent 'sin wave' to the center of all 5 lanes
                % Snap the target CTE to the closest lane based on a 3.5m lane width
                current_lane_offset = round(cte_horizon(1) / 3.5) * 3.5;
                snapped_horizon = cte_horizon - current_lane_offset;
                
                % --- NMPC LANE FOLLOWING ---
                % Generate optimal steering to minimize CTE over the horizon
                last_steer = lane_following_nmpc(state, snapped_horizon, speed_cmd, last_steer);
                
                % --- VISUALIZER UPDATES ---
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

function [has_data, state, laser, lane_horizons, cte_horizon] = read_udp_sensors(u, expectedBytes, expectedDoubles)
    has_data = false;
    state = []; laser = []; lane_horizons = []; cte_horizon = [];
    
    % Flush buffer to get latest packet
    while u.NumBytesAvailable >= expectedBytes
        all_data = read(u, expectedDoubles, "double");
        has_data = true;
    end
    
    if has_data && ~any(isnan(all_data))
        state         = all_data(1:13);
        laser         = all_data(14:373);
        lane_horizons = reshape(all_data(374:693), [64, 5])'; % 5 rows, 64 cols
        cte_horizon   = all_data(694:698);
    else
        has_data = false; % Filter out NaN traces
    end
end

function opt_steer = lane_following_nmpc(state, cte_horizon, v, last_steer)
    % NMPC Formulation over 5 steps
    % Godot car physics: Unicycle model where 'steer' = yaw rate (omega) rad/s
    % We want to find a sequence of omega [w1, w2, w3, w4, w5] that minimizes future CTE
    
    N = 5;
    dt = 5.0 / v; % Time to travel 5 meters (lookahead spacing)
    
    % Objective Function
    obj_fun = @(w) compute_nmpc_cost(w, cte_horizon, dt, v, last_steer);
    
    % Constraints (Max yaw rate)
    lb = -1.5 * ones(N, 1);
    ub =  1.5 * ones(N, 1);
    
    % Initial Guess (Constant yaw rate)
    w0 = last_steer * ones(N, 1);
    
    % Optimize
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    w_opt = fmincon(obj_fun, w0, [], [], [], [], lb, ub, [], options);
    
    % Output the first control action
    opt_steer = w_opt(1);
end

function cost = compute_nmpc_cost(w, cte_ref, dt, v, last_steer)
    N = length(w);
    cost = 0;
    
    % Current local state relative to road
    y = 0;     % Lateral deviation from current car position (starts at 0)
    theta = 0; % Heading deviation (starts at 0)
    
    W_cte = 100.0; % Weight for tracking the road center
    W_u = 5.0;     % Weight for steering magnitude
    W_du = 50.0;   % Weight for steering smoothness (Increased to reduce oscillation)
    
    prev_w = last_steer;
    
    for k = 1:N
        % Physics integration (Unicycle)
        theta = theta + w(k) * dt;
        y = y + v * sin(theta) * dt;
        
        % Error against true road CTE. 
        % The lookahead cte_ref(k) is the offset of the ROAD relative to the STARTING car frame.
        % So to minimize displacement from center, our predicted y should match cte_ref(k).
        err = y - cte_ref(k);
        
        cost = cost + W_cte * (err^2) + W_u * (w(k)^2) + W_du * ((w(k) - prev_w)^2);
        prev_w = w(k);
    end
end