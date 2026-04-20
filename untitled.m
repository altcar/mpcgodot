%% MATLAB UDP Controller, IMU Monitor & 360 LiDAR Visualizer
% Configuration:
% - Expected Packet: 14 (IMU) + 360 (Laser) = 374 doubles
% - Byte Size: 374 * 8 = 2992 bytes

% 1. CLEANUP: Force close any existing ports
delete(findall(0, "Type", "udpport")); 
clear u;

% 2. INITIALIZATION
godotIP = "127.0.0.1";
godotPort = 4246;
matlabPort = 4247;
expectedDoubles = 374; 
expectedBytes = expectedDoubles * 8;

u = udpport("LocalPort", matlabPort);

% Setup Polar Plot for Laser Visualization
figure('Name', 'Car 360 Laser Scan', 'NumberTitle', 'off');
theta = deg2rad(0:359);
hPolar = polarplot(theta, zeros(1, 360), 'LineWidth', 1.5);
title('LiDAR 360° Scan (Distance in Meters)');
rlim([0 50]); % Match the max_range in your Godot script
grid on;

fprintf("====================================================\n");
fprintf("MATLAB: Full State Monitor & LiDAR Scan Active\n");
fprintf("Expecting %d bytes per packet\n", expectedBytes);
fprintf("====================================================\n\n");

loop_count = 0;

try
    while true
        loop_count = loop_count + 1;
        
        % 3. GENERATE CONTROL COMMANDS
        t_now = datetime("now");
        steer_cmd = 0.5 * sin(second(t_now)); 
        speed_cmd = 15.0;                     
        
        % 4. SEND TO GODOT (Steering and Speed)
        write(u, [steer_cmd, speed_cmd], "double", godotIP, godotPort);
        
        % 5. RECEIVE FULL DATA VECTOR (IMU + LASER)
        if u.NumBytesAvailable >= expectedBytes
            % Read all 374 doubles at once
            all_data = read(u, expectedDoubles, "double");
            
            if any(isnan(all_data))
                fprintf("WARNING [RX]: Received NaN data!\n");
            else
                % --- DATA MAPPING ---
                % Part A: IMU (Indices 1 to 14)
                time_stamp = all_data(1);
                pos        = all_data(2:4);   
                quat       = all_data(5:8);   
                lin_vel    = all_data(9:11);  
                ang_vel    = all_data(12:14); 
                
                % Part B: Laser Scan (Indices 15 to 374)
                laser_scan = all_data(15:374);
                
                % --- VISUALIZER ---
                % Update the polar plot with the new laser data
                set(hPolar, 'YData', laser_scan);
                drawnow limitrate; % High performance update
                
                % --- DEBUG PRINTOUT ---
                % Prints IMU state and the distance to the closest object
                fprintf("T:%6.2f | P:[%5.1f %5.1f %5.1f] | V:%5.1f m/s | Min_Dist:%4.1f m | STEER:%5.2f\n", ...
                    time_stamp, ...
                    pos(1), pos(2), pos(3), ...
                    norm(lin_vel), ...
                    min(laser_scan), ...
                    steer_cmd);
            end
            
        elseif u.NumBytesAvailable > 0 && u.NumBytesAvailable < expectedBytes
            % This happens if the packets are getting clipped
            fprintf("DEBUG [RX]: Buffer Mismatch (%d/%d bytes). Flushing...\n", ...
                u.NumBytesAvailable, expectedBytes);
            flush(u);
        end
        
        pause(0.01); 
    end
    
catch ME
    fprintf("\n[STOPPED]: %s\n", ME.message);
    clear u;
end