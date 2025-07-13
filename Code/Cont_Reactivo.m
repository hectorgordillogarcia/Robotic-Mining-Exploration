function [velocidadLineal, velocidadAngular, hayObstaculo] = Cont_Reactivo()

    % Obtención de datos de los sensores ultrasónicos
    lecturasSensores = apoloGetAllultrasonicSensors('Marvin');
    sensorIzquierdo = lecturasSensores(2);
    sensorFrontal = lecturasSensores(1);
    sensorDerecho = lecturasSensores(3);
    
    % Inicialización de velocidades
    velocidadLineal = 0;
    velocidadAngular = 0;
    hayObstaculo = false;

    % Comprobación de detección de obstáculo
    if (sensorFrontal < 1) || (sensorDerecho < 0.5) || (sensorIzquierdo < 0.5)
        hayObstaculo = true;

        if (sensorDerecho < 0.2) || (sensorIzquierdo < 0.2) % Obstáculo muy cercano en los lados
            velocidadLineal = 0.05;
            if sensorIzquierdo < 0.2 % Obstáculo en el lado izquierdo
                velocidadAngular = -0.4 / sensorIzquierdo;
            else % Obstáculo en el lado derecho
                velocidadAngular = 0.4 / sensorDerecho;
            end

        else
            velocidadLineal = 0.1;
            if sensorIzquierdo < sensorDerecho % Obstáculo más cercano por la izquierda
                velocidadAngular = -0.5 / sensorIzquierdo;
            else % Obstáculo más cercano por la derecha
                velocidadAngular = 0.5 / sensorDerecho;
            end
        end

    else
        % Comportamiento sin obstáculos
        velocidadAngular = 1;
        velocidadLineal = 1;
    end
end
