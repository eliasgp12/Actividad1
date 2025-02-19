% Limpieza de pantalla
clear;
close all;
clc;

% Declaración de variables simbólicas
syms th1(t) th2(t) l1 l2 t  

% Configuración del robot (0 = rotacional, 1 = prismática)
RP = [0, 0];

% Vector de coordenadas articulares
Q = [th1; th2];
disp('Coordenadas articulares');
pretty(Q);

% Vector de velocidades articulares
Qp = diff(Q, t);
disp('Velocidades articulares');
pretty(Qp);

% Número de grados de libertad
GDL = size(RP, 2);

% Articulación 1
P(:,:,1) = [l1*cos(th1);
           l1*sin(th1);
           0];
R(:,:,1) = [cos(th1) -sin(th1) 0;
           sin(th1)  cos(th1) 0;
           0         0        1];

% Articulación 2
P(:,:,2) = [l2*cos(th1 + th2);
           l2*sin(th1 + th2);
           0];
R(:,:,2) = [cos(th1 + th2) -sin(th1 + th2) 0;
           sin(th1 + th2)  cos(th1 + th2) 0;
           0              0             1];

% Inicialización de matrices de transformación homogénea
Vector_Zeros = zeros(1, 3);
A(:,:,1) = simplify([R(:,:,1) P(:,:,1); Vector_Zeros 1]);
A(:,:,2) = simplify([R(:,:,2) P(:,:,2); Vector_Zeros 1]);

% Transformaciones globales
T(:,:,1) = A(:,:,1);
T(:,:,2) = simplify(T(:,:,1) * A(:,:,2));

% Posiciones y rotaciones globales
PO(:,:,1) = P(:,:,1);
PO(:,:,2) = T(1:3,4,2);
RO(:,:,1) = R(:,:,1);
RO(:,:,2) = T(1:3,1:3,2);

% Cálculo del Jacobiano
Jv_a = sym(zeros(3, GDL));
Jw_a = sym(zeros(3, GDL));

for k = 1:GDL
    if RP(k) == 0 % Articulación rotacional
        if k == 1
            Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));
            Jw_a(:,k) = [0;0;1];
        else
            Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL) - PO(:,:,k-1));
            Jw_a(:,k) = RO(:,3,k-1);
        end
    else % Articulación prismática
        Jv_a(:,k) = RO(:,3,k-1);
        Jw_a(:,k) = [0;0;0];
    end
end

% Resultados finales
Jv_a = simplify(Jv_a);
Jw_a = simplify(Jw_a);
disp('Jacobiano lineal analítico');
pretty(Jv_a);
disp('Jacobiano angular analítico');
pretty(Jw_a);

V = simplify(Jv_a * Qp);
disp('Velocidad lineal');
pretty(V);

W = simplify(Jw_a * Qp);
disp('Velocidad angular');
pretty(W);
