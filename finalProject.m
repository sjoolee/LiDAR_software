clc;
clear;

s = serialport("COM6",115200);
configureTerminator(s,"CR/LF");
message = sprintf("Opening %s", s.Port);
flush(s);

data = [];

while true
    message = getUART(s);
    if strcmp(message, "Starting to measure")
        deg_per_measurement = str2double(getUART(s));
        num_of_measurements = 360/deg_per_measurement;
        YZ_Data = zeros(1,num_of_measurements);
        
        while true
            message = getUART(s);
            if strcmp(message, "Ending measurement process")
                message;
                message = getUART(s);
                break;
            else
                str = split(message)';
                YZ_Data = str2double(str); % Vectorized conversion
                data = [data ; YZ_Data];
            end
        end
        
        try
            base_ang = 90:deg_per_measurement:450-deg_per_measurement;
            angles = repmat(base_ang, 1, length(data(:,1))); % Repeat baseAngles for each data point
            angles = deg2rad(angles);
            
            x = (0:length(data(:,1))-1) * 10;
            y = data .* sin(angles);
            z = data .* cos(angles);
            
            hold on
            plot3(xPoints, zPoints, yPoints, "color", "blue");
            hold off
            xlabel("X")
            ylabel("Z")
            zlabel("Y")
            xlim([-30 max(max(x))+30])
            ylim([-5000 5000])
            zlim([-5000 5000])
        catch
            message = "No Data Collected";
        end
    end
end

function UART_MSG = getUART(serialPort)
    warning("on", "all");
    warning("off", "serialport:ReadlineWarning");
    while true
        lastwarn("", "");
        UART_MSG = readline(serialPort);
        [warnMSG, warnID] = lastwarn();
        if warnID ~= "serialport:ReadlineWarning"
            break
        end
    end
    warning("on", "all");
end