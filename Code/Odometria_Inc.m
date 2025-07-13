function [var_desplazamiento,var_rotacion, var_coord_x, var_coord_y, var_orientacion]=Odometria_Inc()
    vel_lineal = 0.2; % m/s
    vel_angular = 0.2; % rad/s
    intervalo_tiempo = 0.5; % s

    num_iteraciones = 300; %Número de iteraciones
    desplazamiento_odometria = zeros(num_iteraciones,1);
    rotacion_odometria = zeros(num_iteraciones,1);
    desplazamiento_real = zeros(num_iteraciones,1);
    rotacion_real = zeros(num_iteraciones,1);

    % Configurar posición inicial del robot y orientación
    apoloPlaceMRobot('Marvin',[0 0 0],0);
    apoloUpdate();
    localizacion_real = apoloGetLocationMRobot('Marvin');
    posiciones_reales =[localizacion_real(1),localizacion_real(2),localizacion_real(4)];

    % Resetear la odometría
    apoloResetOdometry('Marvin');
    localizacion_odometrica = apoloGetOdometry('Marvin');

    % Mover el robot y registrar datos de odometría
    for j = 1:num_iteraciones
        apoloMoveMRobot('Marvin',[vel_lineal, vel_angular], intervalo_tiempo);
        apoloUpdate();

        % Capturar datos de la odometría
        pos_odometrica_actual = apoloGetOdometry('Marvin');
        localizacion_odometrica = [localizacion_odometrica; pos_odometrica_actual];

        % Calcular desplazamiento y rotación por odometría
        desplazamiento_odometria(j,1) = sqrt((localizacion_odometrica(j+1,1)-localizacion_odometrica(j,1))^2 + (localizacion_odometrica(j+1,2)-localizacion_odometrica(j,2))^2);
        rotacion_odometria(j,1) = abs(localizacion_odometrica(j+1,3))-abs(localizacion_odometrica(j,3));

        % Capturar datos de la posición real
        localizacion_actual = apoloGetLocationMRobot('Marvin');
        posiciones_reales = [posiciones_reales; localizacion_actual(1),localizacion_actual(2),localizacion_actual(4)];

        % Calcular desplazamiento y rotación reales
        desplazamiento_real(j,1) = sqrt((posiciones_reales(j+1,1)-posiciones_reales(j,1))^2 + (posiciones_reales(j+1,2)-posiciones_reales(j,2))^2);
        rotacion_real(j,1) = abs(posiciones_reales(j+1,3))-abs(posiciones_reales(j,3));
    end

    % Calcular errores y varianzas de las posiciones
    errores_pos = abs(localizacion_odometrica - posiciones_reales);
    errores_pos(:,3) = mod(errores_pos(:,3),pi); %Asegurar el error entre 0 y pi

    desviaciones_std = std(errores_pos);
    var_coord_x = (desviaciones_std(1))^2;
    var_coord_y = (desviaciones_std(2))^2;
    var_orientacion = (desviaciones_std(3))^2;

    % Calcular medias y varianzas del desplazamiento y rotación
    errores_desplazamiento = abs(desplazamiento_odometria-desplazamiento_real);
    var_desplazamiento = (std(errores_desplazamiento))^2;

    errores_rotacion = abs(rotacion_odometria-rotacion_real);
    var_rotacion = (std(errores_rotacion))^2;
end
