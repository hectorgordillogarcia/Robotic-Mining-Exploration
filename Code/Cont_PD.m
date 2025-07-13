function [vel_lineal, vel_angular, objetivo_alcanzado] = Cont_PD(tiempoRefresco,estado_actual, objetivo)
K_proporcional = 1.0;
K_derivativo = 0.4;
tiempo_muestreo = tiempoRefresco;
velocidad_lineal_min = 0.2;   % Velocidad lineal mínima
umbral_angular = pi/12;     % Umbral mínimo de ajuste angular
limite_vel_angular = 1;     % Saturación de velocidad angular


%% Variables persistentes para errores
persistent error_previo 
if isempty(error_previo) 
    error_previo = 0;
end

%% Cálculos del controlador
% Cálculo del ángulo hacia el objetivo
angulo_deseado = atan2(objetivo(2) - estado_actual(2), objetivo(1) - estado_actual(1));

% Normalización del ángulo entre -pi y pi
error_angular = angulo_deseado - estado_actual(3);
error_angular = mod(error_angular + pi, 2*pi) - pi; % Ajuste en rango [-pi, pi]

% Cálculo de los errores P y D
error_p = sin(error_angular);
error_d = (error_p - error_previo) / tiempo_muestreo;

% Controlador PD para la velocidad angular
vel_angular = K_proporcional * error_p+ K_derivativo * error_d;

% Saturación de la velocidad angular
vel_angular = max(-limite_vel_angular, min(limite_vel_angular, vel_angular));

% Actualización del error previo
error_previo = error_p;

%% Ajuste de la velocidad lineal
if abs(error_angular) < umbral_angular
    vel_lineal = 1; % Velocidad máxima cuando el ángulo es pequeño
else
    vel_lineal = velocidad_lineal_min; % Velocidad reducida para giros
end

%% Verificación de si el objetivo ha sido alcanzado
distancia_al_objetivo = sqrt((objetivo(2) - estado_actual(2))^2 + (objetivo(1) - estado_actual(1))^2);
umbral_distancia = 0.3; % Definir umbral para considerar que se ha alcanzado el objetivo
objetivo_alcanzado = (distancia_al_objetivo < umbral_distancia);

end
