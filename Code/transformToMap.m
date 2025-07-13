function [x_original, y_original] = transformToMap1(sentido, x_new, y_new)
    % Función para transformar coordenadas entre dos mapas en ambos sentidos
    % Entrada:
    %   sentido - Direccion de la transformación (1 o 2):
    %             1 -> Calcular x_new, y_new a partir de x_original, y_original
    %             2 -> Calcular x_original, y_original a partir de x_new, y_new
    %   x_new - Coordenada X (según el sentido)
    %   y_new - Coordenada Y (según el sentido)
    % Salida:
    %   x_original - Coordenada X transformada (según el sentido)
    %   y_original - Coordenada Y transformada (según el sentido)

    % Escalas
    scale_x = 39 / 547;
    scale_y = 32 / 547;

    if sentido == 1
        % Transformar de mapa 1 a mapa 2 (x_original -> x_new)
        x_original = (x_new + 2) / scale_x;
        y_original = (y_new + 16) / scale_y;
    elseif sentido == 2
        % Transformar de mapa 2 a mapa 1 (x_new -> x_original)
        x_original = (x_new * scale_x) - 2;
        y_original = (y_new * scale_y) - 16;
    else
        error('El valor de "sentido" debe ser 1 o 2.');
    end
end


