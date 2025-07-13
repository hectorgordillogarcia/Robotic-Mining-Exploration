function [var_d, var_a, var_u] = Sensores_Inc()

robot_name = 'Marvin';
robot_position = [0, 0, 0]; % Posición inicial

% Colocar el robot en la posición inicial
apoloPlaceMRobot(robot_name, robot_position, robot_position(3));
apoloUpdate();

% Calibración del sensor láser
laser_data = apoloGetLaserLandMarks('LMS100');
laser_distances = laser_data.distance;
laser_angles = laser_data.angle;

for k = 1:4000 
    laser_data = apoloGetLaserLandMarks('LMS100');
    laser_distances = [laser_distances; laser_data.distance];
    laser_angles = [laser_angles; laser_data.angle];
    apoloUpdate();
end

% Cálculo de las varianzas
var_d = var(laser_distances);
var_a = var(laser_angles, 1);

% Calibración de los sensores ultrasónicos
ultrasonic_data = apoloGetAllultrasonicSensors(robot_name);

for k = 1:4000
    ultrasonic_data = [ultrasonic_data; apoloGetAllultrasonicSensors(robot_name)];
    apoloUpdate();
end

% Cálculo de la varianza
var_u = var(ultrasonic_data);

end
