clear all
clc
close all

% Parámetros de la simulación
tiempoRefresco = 0.20; % Frecuencia de actualización
tiempoMaximo = 500;    % Tiempo máximo de simulación
umbralWaypoint = 1; % Distancia al waypoint para considerarlo alcanzado

% Parámetros de velocidad
velMaxLineal = 0.40; % Velocidad lineal máxima (m/s)
velMaxAngular = 1.5; % Velocidad angular máxima (rad/s)

% Configuración del entorno
inicio = [0, 0]; % Coordenadas del inicio
objetivo = [36, 0]; % Coordenadas del objetivo

%Transformamos las coordenadas de apolo a matlab
[objetivo(1), objetivo(2)] = transformToMap(1,objetivo(1), objetivo(2));
[inicio(1), inicio(2)] = transformToMap(1,inicio(1), inicio(2));

% Parámetros del robot
robot = 'Marvin';
posicionRobot = [0, 0, 0]; % Posición inicial

% Posicionamiento inicial
apoloPlaceMRobot(robot, posicionRobot, posicionRobot(3));
apoloUpdate();

% Planificación inicial
[waypoints] = PlanificadorRRT(inicio, objetivo);

% Transformar los waypoints de coordenadas de matlab a apolo
mapa_waypoints = zeros(size(waypoints)); % Inicializar matriz para los waypoints transformados
    for i = 1:size(waypoints, 1)
        [mapa_waypoints(i, 1), mapa_waypoints(i, 2)] = transformToMap(2,waypoints(i, 1), waypoints(i, 2));
    end
waypoints = mapa_waypoints;


% Bucle de navegación
tiempoSimulacion = 0;
waypointActual = 1;
iteracion = 0;

while tiempoSimulacion < tiempoMaximo
    iteracion = iteracion + 1;

    % Actualizar estimación de posición (Filtro de Kalman)
    [estadoReal, estadoEstimado] = Kalman_EXT();  


    % Calcular distancia y ángulo al waypoint actual
    distanciaObjetivo = norm(waypoints(waypointActual, 1:2) - estadoEstimado(1:2)');
    anguloObjetivo = atan2(waypoints(waypointActual, 2) - estadoEstimado(2),waypoints(waypointActual, 1) - estadoEstimado(1));

    % Avanzar al siguiente waypoint si se ha alcanzado el actual
    while distanciaObjetivo < umbralWaypoint && waypointActual < length(waypoints(:, 1))
        waypointActual = waypointActual + 1;
        distanciaObjetivo = norm(waypoints(waypointActual, 1:2) - estadoEstimado(1:2)');
        anguloObjetivo = atan2(waypoints(waypointActual, 2) - estadoEstimado(2),waypoints(waypointActual, 1) - estadoEstimado(1));
    end

    % Control PID para seguir la trayectoria
    [velLinealPID, velAngularPID, objetivoAlcanzado] = Cont_PD(tiempoRefresco,estadoEstimado, waypoints(waypointActual, :));

    % Verificar si se alcanzó el objetivo final
    if waypointActual >= length(waypoints(:, 1)) && objetivoAlcanzado
        disp('Se ha alcanzado el objetivo');
        break;
    end

    % % Control reactivo para evitar colisiones
    [velReactivaLineal, velReactivaAngular, obstaculo] = Cont_Reactivo();
   

    % Combinar controles de trayectoria y reactivo
    if obstaculo
        velLineal = velReactivaLineal* velMaxLineal;
        velAngular = velReactivaAngular* velMaxAngular;
    else % el sistema prioriza el seguimiento de la trayectoria (control PID), 
        % pero conserva un pequeño componente reactivo como medida de seguridad
        % ante cambios inesperados en el entorno.
        ponderacionPID = 2/3; % Peso del control PID
        ponderacionReactiva = 1/3; % Peso del control reactivo
        
        velLineal = (ponderacionPID * velLinealPID + ponderacionReactiva * velReactivaLineal) * velMaxLineal;
        velAngular = (ponderacionPID * velAngularPID + ponderacionReactiva * velReactivaAngular) * velMaxAngular;

    end
    apoloMoveMRobot(robot, [velLineal, velAngular], tiempoRefresco);
    apoloUpdate();

    % Actualizar tiempo y entorno
    tiempoSimulacion = tiempoSimulacion + tiempoRefresco;    
    pause(0.02);

end
