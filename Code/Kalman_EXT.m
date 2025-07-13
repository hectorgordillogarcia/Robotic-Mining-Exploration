function [X_real,Xk]=Kalman_EXT()

% Posicion ACtual
posicionActual = apoloGetLocationMRobot('Marvin');
Xk_1 = [posicionActual(1), posicionActual(2), posicionActual(4)];  % Sincronizar estado inicial

% Varianza en odometría (calibración odometría) 
var_x = 6.1794e-04;
var_y = 7.8640e-05;
var_theta = 2.8124e-05; 
Pk_1 = [var_x 0 0;
      0 var_y 0;
      0 0 var_theta];

% Reinicia la odometría para evitar desplazamientos acumulados
apoloResetOdometry('Marvin', [posicionActual(1), posicionActual(2), posicionActual(4)]);

    % Varianza del ruido del proceso
    Qk = [1.1282e-05 0;
                0 2.2784e-06];

    % Varianza en la medida (calibración del laser)
    Rk = [4.4388e-04 0 0;
          0 4.4388e-04 0;
          0 0 3.0514e-04];

    % Coordenadas de las balizas:
    Balizas = [1.9   0.0;   
               1.9   5.2;  
              10.9   2.8;   
              10.9   6.8;   
               7.5   1.1;   
               9.0  -8.9;   
              17.5  -7.1;   
              22.9 -10.0;   
              21.0   1.9;   
              25.0  -0.9;   
              32.9  -3.0;   
              32.9   3.0;   
              30.0   6.9;   
              36.9   1.0];  

% Posicion actual
    posicionActual = apoloGetLocationMRobot('Marvin');
    X_real = [posicionActual(1) posicionActual(2) posicionActual(4)];
    
    % Empleo de sensores laser para identificar las balizas
    sensores_laser = apoloGetLaserLandMarks('LMS100');
    total_balizas = length(sensores_laser.id); 

    %Predecimos el movimiento del robot
        movimiento = apoloGetOdometry('Marvin');
        lineal = sqrt((Xk_1(1)-movimiento(1))^2+(Xk_1(2)-movimiento(2))^2);
        angular = movimiento(3)-Xk_1(3);
        
    % Prediccion del estado
    X_k = [(Xk_1(1) + lineal*cos(Xk_1(3)+(angular/2)));
           (Xk_1(2) + lineal*sin(Xk_1(3)+(angular/2)));
           (Xk_1(3) + angular)];

    Ak = [1 0 (-lineal*sin(Xk_1(3)+angular/2));
          0 1 (lineal*cos(Xk_1(3)+angular/2));
          0 0 1];

    Bk = [(cos(Xk_1(3)+angular/2)) (-0.5*lineal*sin(Xk_1(3)+angular/2));
          (sin(Xk_1(3)+angular/2)) (0.5*lineal*cos(Xk_1(3)+angular/2));
           0 1];

    P_k = Ak*Pk_1*((Ak)') + Bk*Qk*((Bk)');
    if total_balizas >=2                  

        Dx1=Balizas(sensores_laser.id(1),1)-X_k(1);
        Dy1=Balizas(sensores_laser.id(1),2)-X_k(2);

        Dx2=Balizas(sensores_laser.id(2),1)-X_k(1);
        Dy2=Balizas(sensores_laser.id(2),2)-X_k(2);


        vecY_norm=sin(atan2(Dy2,Dx2));
        vecX_norm =cos(atan2(Dy2,Dx2));
        AjusteAngular=1/(1+(Dy2/Dx2)^2);

    % Prediccion de la medida
        Zk_ = [atan2(Dy1,Dx1)-X_k(3);
               atan2(Dy2,Dx2)-X_k(3);
               Dy2/vecY_norm];

        Zk = [sensores_laser.angle(1);
              sensores_laser.angle(2);
              sensores_laser.distance(2)];

        Hk = [((Dy1)/((Dx1)^2+(Dy1)^2)) (-(Dx1)/((Dx1)^2+(Dy1)^2)) (-1);
              ((Dy2)/((Dx2)^2+(Dy2)^2)) (-(Dx2)/((Dx2)^2+(Dy2)^2)) (-1);   
              ((-Dy2)*(vecX_norm )*AjusteAngular*(Dy2/(Dx2^2))/vecY_norm^2) (((-vecY_norm)-(Dy2*(vecX_norm )*AjusteAngular*(-1/Dx2)))/vecY_norm^2) (0) ];
    
        % Comparacion
        Yk = Zk-Zk_;
      
           for r=1:3
                if Yk(r)>pi
                    Yk(r) = Yk(r) - 2*pi;
                end
                if Yk(r)<(-pi)
                    Yk(r) = Yk(r) + 2*pi;
                end
            end
            Yk;
            Sk = Hk*P_k*((Hk)') + Rk;
            Wk = P_k*((Hk)')*inv(Sk);

            %Correccion
            Xk = X_k + Wk*Yk;
            Pk = (eye(3)-Wk*Hk)*P_k;   
    else 
        Xk=X_k;
        Pk=Pk_1;
    end
    apoloResetOdometry('Marvin', [Xk(1) Xk(2) Xk(3)]);
end