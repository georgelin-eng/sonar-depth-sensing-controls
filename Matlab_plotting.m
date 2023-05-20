s = serialport("COM8", 9600);  % Replace "COM8" with the appropriate serial port of your Arduino
configureTerminator(s, "LF");

altitudeData = [];  % Initialize an empty array to store the altitude values

while true
    line = readline(s);
    altitude = str2double(line);
    
    if ~isnan(altitude)
        altitudeData = [altitudeData; altitude];
        
        % Plot the altitude values
        plot(altitudeData);
        xlabel('Data Points');
        ylabel('Altitude');
        title('Altitude Plot');
        drawnow;  % Update the plot
        
        % Optional: Save the altitude values to a file
        % save('altitude_data.mat', 'altitudeData');
    end
end
