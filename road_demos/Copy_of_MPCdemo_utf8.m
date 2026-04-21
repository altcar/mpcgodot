function F1_MPC_Full_DualView()
    clc; clear; close all;
    
    %% 1. System Matrices (State-Space)
    ts = 0.01;
    ad = [1.00027180509492,0.00625101658363726,-0.000298104527325984,-0.000592137149727941,-0.000195218555764740;-0.00625101658365004,0.879670596217866,0.0123995907573806,0.00942892684037583,-0.00775386215642799;-0.000298104527325549,-0.0123995907573839,0.999169855139624,-0.0148759276100900,0.000129671924415677;0.000592137149728420,0.00942892684037156,0.0148759276100894,0.998913472148301,0.0286900249744246;-0.000195218555764543,0.00775386215643324,0.000129671924425366,-0.0286900249744255,0.999703452784522];
    bd = [-0.023307871208778;-0.314731276263951;-0.008803109981206;0.016810972019614;0.005019051193557];
    cs = [0.023307871208772,-0.314731276263952,0.008803109981209,0.016810972019614,-0.005019051193548];
    sys1 = ss(ad,bd,cs,0,ts);

    %% 2. MPC Parameters & Weight Matrices
    P = 10; M = 5; % Horizon lengths
    q = 150; r = 1; h = 0.5; alpha = 0.1;
    
    [step_res,~] = step(sys1, ts:ts:P*ts);
    A = zeros(P,M); A(:,1) = step_res(1:P);
    for j=2:M, A(:,j) = [zeros(j-1,1); A(1:P-j+1, 1)]; end
    
    Q = eye(P)*q; R = eye(M)*r; H = ones(P,1)*h;
    S = zeros(P,P); for i=1:P-1, S(i,i+1)=1; end, S(P,P)=1;

    %% 3. Track Generation & Random Obstacle
    theta = linspace(0, 2*pi, 1000);
    r_f1 = 65 + 15*sin(3*theta) + 5*cos(5*theta);
    track_x = r_f1 .* cos(theta);
    track_y = r_f1 .* sin(theta);
    map = [track_x', track_y'];
    
    % Randomly place obstacle index on the track
    obs_idx = randi([200, 700]);
    obs_pos = map(obs_idx, :);
    obs_offset = 0.0; % 2 meters to the side in 1D space

    %% 4. Setup Visualization (Dual View)
    figure(1); set(gcf, 'Position', [50 100 1300 500], 'Color', 'w');
    
    % --- Subplot 1: 2D Global Map ---
    subplot(1,2,1); hold on; axis equal; grid on;
    plot(track_x, track_y, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Reference Track');
    h_path_2d = plot(nan, nan, 'b', 'LineWidth', 2, 'DisplayName', 'Actual Path');
    h_obs_2d  = plot(obs_pos(1), obs_pos(2), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 10, 'DisplayName', 'Obstacle');
    h_car_2d  = plot(track_x(1), track_y(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    title('2D F1 Track Navigation'); legend('Location', 'best');

    % --- Subplot 2: 1D MPC Logic ---
    subplot(1,2,2); hold on; grid on;
    h_err_hist = plot(nan, nan, 'b', 'LineWidth', 2, 'DisplayName', 'Historic Error');
    h_err_pred = plot(nan, nan, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Optimal Plan');
    h_obs_1d   = plot(obs_idx, obs_offset, 'mo', 'MarkerFaceColor', 'm', 'DisplayName', 'Obstacle Gate');
    xlabel('Track Index (k)'); ylabel('Lateral Offset (m)');
    title('1D Lateral Control & Avoidance'); ylim([-4 4]); legend;

    %% 5. Simulation Loop
    pos = [track_x(1); track_y(1)]; 
    yaw = theta(1) + pi/2; 
    v = 15; % Speed
    Y0 = zeros(P,1); 
    sim_len = 900;
    pos_hist = zeros(sim_len, 2); 
    err_hist = zeros(sim_len, 1);

    for k = 1:sim_len
        % A. Calculate current 1D Lateral Error (CTE)
        dists = sqrt((map(:,1)-pos(1)).^2 + (map(:,2)-pos(2)).^2);
        [~, current_idx] = min(dists);
        
        % Vector-based error calculation
        idx_next = mod(current_idx, 1000) + 1;
        tangent = map(idx_next, :) - map(current_idx, :);
        to_car  = pos' - map(current_idx, :);
        current_y = (to_car(1)*tangent(2) - to_car(2)*tangent(1)) / norm(tangent);
        err_hist(k) = current_y;

        % B. Set Reference (Avoidance Logic)
        W = zeros(P,1);
        for i = 1:P
            look_ahead = current_idx + i;
            % If the car is approaching the obstacle's index
            if abs(look_ahead - obs_idx) < 10
                W(i) = obs_offset; 
            else
                W(i) = 0; % Stay on center line
            end
        end

        % C. MPC Solve
        Y_cor = Y0 + H * (current_y - Y0(1,1));
        Y0_shifted = S * Y_cor;
        
        H_mat = 2 * (A' * Q * A + R);
        f_vec = -2 * A' * Q * (W - Y0_shifted);
        DU = quadprog(H_mat, f_vec, [], [], [], [], -0.8*ones(M,1), 0.8*ones(M,1), [], optimoptions('quadprog','Display','off'));
        if isempty(DU), DU = zeros(M,1); end
        
        u_steer = DU(1);
        Y0 = Y0_shifted + A * DU;

        % D. Update 2D Physics
        yaw = yaw + (u_steer * 0.35); % Steering sensitivity
        pos(1) = pos(1) + v * cos(yaw) * 0.05;
        pos(2) = pos(2) + v * sin(yaw) * 0.05;
        pos_hist(k,:) = pos;

        % E. Synchronize Visuals
        set(h_path_2d, 'XData', pos_hist(1:k,1), 'YData', pos_hist(1:k,2));
        set(h_car_2d, 'XData', pos(1), 'YData', pos(2));
        
        set(h_err_hist, 'XData', 1:k, 'YData', err_hist(1:k));
        set(h_err_pred, 'XData', k:(k+P-1), 'YData', Y0);
        
        if mod(k,5)==0, drawnow limitrate; end
    end
end