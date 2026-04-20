% Configuration
godotIP = "127.0.0.1";
godotPort = 4246;
matlabPort = 4247;

clear u;
% Create UDP object
u = udpport("LocalPort", matlabPort);

disp("Transceiver started. Press Ctrl+C to stop.");

try
    while true
        % 1. Send data
        t = datetime("now","Format","ss.SSSS");
        val = sin(second(t)); 
        write(u, val, "double", "127.0.0.1", 4246);
        
        % 2. Check for incoming data
        % Use a small while loop to catch everything in the buffer
        while u.NumBytesAvailable > 0
            rawData = read(u, u.NumBytesAvailable, "uint8");
            charData = char(rawData);
            fprintf("Godot says: %s\n", charData);
        end
        
        pause(0.01); % Slow it down slightly to 10Hz just for testing
    end
catch
    clear u;
end
        