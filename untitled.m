% --- Cleanup existing ports ---
clear u;
% --- Initialization ---
u = udpport("LocalPort", 4247);
fprintf("MATLAB: Full IMU Stream Started...\n");
fprintf("Format: [Time, PosX, PosY, PosZ, QuatW, QuatX, QuatY, QuatZ, VelX, VelY, VelZ, AngX, AngY, AngZ]\n");
fprintf("------------------------------------------------------------------------------------------------\n");

try
    while true
        % 1. CONSTANT SEND
        t_now = datetime("now", "Format", "ss.SSSS");
        val = sin(second(t_now)); 
        write(u, val, "double", "127.0.0.1", 4246);
        
        % 2. RECEIVE ALL 14 DOUBLES
        if u.NumBytesAvailable >= 112
            data = read(u, 14, "double");
            
            % Print the entire vector on one line
            % %7.2f provides a fixed width so the columns don't jump around
            fprintf("T:%6.2f | P:[%5.1f %5.1f %5.1f] | Q:[%4.2f %4.2f %4.2f %4.2f] | V:[%5.1f %5.1f %5.1f] | W:[%4.2f %4.2f %4.2f]\n", ...
                data(1), ...             % Timestamp
                data(2), data(3), data(4), ... % Position
                data(5), data(6), data(7), data(8), ... % Quaternion (W, X, Y, Z)
                data(9), data(10), data(11), ... % Linear Velocity
                data(12), data(13), data(14));   % Angular Velocity
        end
        
        pause(0.01); 
    end
catch ME
    fprintf("\nStopped: %s\n", ME.message);
    clear u;
end