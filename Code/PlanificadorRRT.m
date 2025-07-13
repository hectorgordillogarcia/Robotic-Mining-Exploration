function [ruta] = PlanificadorRRT(posInicio, posMeta)
%% Configuración General
radiogiroMarvin = 8;

% Configuración RRT*
valDistancia      = 10;
maxIteraciones    = 100000;
maxDistConexion   = 1;

%% Procesamiento del Mapa
imagen = imread('Mapa3.jpeg');
imagenGris = rgb2gray(imagen);
imagenBW = imagenGris < 0.5;
mapaBase = binaryOccupancyMap(imagenBW);

% Expansión del mapa para evitar colisiones
mapaAumentado = copy(mapaBase);
inflate(mapaAumentado, radiogiroMarvin);

%% Planificación RRT* Basada en Árbol
espacioEstados = stateSpaceDubins;
espacioEstados.MinTurningRadius = 0.4;

validadorMapa = validatorOccupancyMap(espacioEstados);
validadorMapa.Map = mapaAumentado;
validadorMapa.ValidationDistance = valDistancia;

espacioEstados.StateBounds = [mapaAumentado.XWorldLimits;mapaAumentado.YWorldLimits;[-pi pi]];

planificadorArbol = plannerRRTStar(espacioEstados, validadorMapa);
planificadorArbol.ContinueAfterGoalReached = false;
planificadorArbol.MaxIterations = maxIteraciones;
planificadorArbol.MaxConnectionDistance = maxDistConexion;

% Configuración de estados inicial y objetivo
estadoInicio = [posInicio(1), posInicio(2), 0];
estadoMeta = [posMeta(1), posMeta(2), 0];
[caminoGenerado, infoRuta] = plan(planificadorArbol, estadoInicio, estadoMeta);

exito = infoRuta.IsPathFound;
if exito
    ruta = [caminoGenerado.States(:,1), caminoGenerado.States(:,2)];
else
    disp('No se encontró un camino válido.');
    ruta = [];
    return;
end

% Visualización del Resultado RRT*
figure(3);
mapaBase.show;
hold on;
plot(infoRuta.TreeData(:,1), infoRuta.TreeData(:,2), '.-'); % Expansión del árbol
plot(caminoGenerado.States(:,1), caminoGenerado.States(:,2), 'r-', 'LineWidth', 2); % Ruta final
title('RRT* - Planificación de Ruta');

end
