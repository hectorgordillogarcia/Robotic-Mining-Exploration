% Dimensiones del mapa (en píxeles)
map_width = 500;  % Ancho del mapa
map_height = 500; % Alto del mapa

% Vértices de los obstáculos
vertices = {
    [-2, -1; 2, -1; 2, -15; -2, -15], 
    [2, 3; 6, 3; 6, -15; 2, -15], 
    [6, 2; 7, 2; 7, -15; 6, -15], 
    [7, 1; 8, 1; 8, -15; 7, -15], 
    [8, -8; 19, -8; 19, -15; 8, -15], 
    [19, -12; 23, -12; 23, -15; 19, -15], 
    [23, -1; 26, -1; 26, -15; 23, -15], 
    [26, -5; 33, -5; 33, -15; 26, -15], 
    [33, -1; 35, -1; 35, -15; 33, -15], 
    [35, -2; 37, -2; 37, -15; 35, -15], 
    [-2, 6; 2, 6; 2, 15; -2, 15], 
    [2, 5; 8, 5; 8, 15; 2, 15], 
    [8, 5; 9, 5; 9, 7; 8, 7], 
    [8, 10; 13, 10; 13, 15; 8, 15], 
    [14, -5; 17, -5; 17, 15; 14, 15], 
    [17, -7; 19, -7; 19, 15; 17, 15], 
    [11, 7; 13, 7; 13, 5; 11, 5], 
    [11, 3; 13, 3; 13, 0; 11, 0], 
    [13, -4; 14, -4; 14, -5; 13, -5], 
    [13, -1; 14, -1; 14, 15; 13, 15], 
    [19, 2; 23, 2; 23, 15; 19, 15], 
    [23, 1; 26, 1; 26, 15; 23, 15], 
    [26, 7; 34, 7; 34, 15; 26, 15], 
    [33, 4; 34, 4; 34, 1; 33, 1], 
    [34, 1; 35, 1; 35, 15; 34, 15], 
    [35, 2; 37, 2; 37, 15; 35, 15]
};

% Escalar las coordenadas a la resolución del mapa
function scaled = scale_coordinates(vertices, x_range, y_range, map_width, map_height)
    scaled = {};
    for i = 1:length(vertices)
        poly = vertices{i};
        scaled_poly = [];
        for j = 1:size(poly, 1)
            x = poly(j, 1);
            y = poly(j, 2);
            % Escalar las coordenadas
            scaled_x = round((x - x_range(1)) / (x_range(2) - x_range(1)) * map_width);
            scaled_y = round((y - y_range(1)) / (y_range(2) - y_range(1)) * map_height);
            scaled_poly = [scaled_poly; scaled_x, map_height - scaled_y]; % Invertir Y
        end
        scaled = [scaled, {scaled_poly}];
    end
end

% Rango de las coordenadas originales del mapa
x_range = [-2, 37];  % Rango en el eje X
y_range = [-15, 15]; % Rango en el eje Y

% Escalar los vértices al rango del mapa
scaled_vertices = scale_coordinates(vertices, x_range, y_range, map_width, map_height);

% Crear un mapa vacío (todo blanco)
map_array = ones(map_height, map_width) * 255;

% Dibujar los polígonos en el mapa directamente en el array
for i = 1:length(scaled_vertices)
    poly = scaled_vertices{i};
    mask = poly2mask(poly(:,1), poly(:,2), map_height, map_width);
    map_array(mask) = 0; % Pinta los polígonos en negro (0)
end

% Guardar la imagen como archivo .pgm
filename = 'Mapa2.png';
imwrite(uint8(map_array), filename, 'png');

disp(['¡Mapa generado correctamente en ', filename, '!']);
map_image = imread('Mapa2.png');

% Mostrar la imagen en una figura
figure;
imshow(map_image, 'InitialMagnification', 'fit'); % Ajusta la escala de visualizació