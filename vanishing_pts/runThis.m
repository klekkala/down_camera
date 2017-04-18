im = imread('img1.jpg');

vp = zeros(3,3);

vp(:,1) = getVanishingPoint(im, 1359, 1134, 1435, 1141, 1357, 1280, 1505, 1285);
vp(:,2) = getVanishingPoint(im, 1437, 1070, 1590, 1096, 1575, 953, 1669, 971);
vp(:,3) = getVanishingPoint(im, 730, 760, 719, 1184, 1367, 331, 1372, 659);

hline = real(cross(vp(:,1), vp(:,2)));
hline = hline ./ norm([hline(1), hline(2)]);
fprintf(1,'The horizontal vanishing line is %fu+%fv+%f=0\n',hline(1), hline(2), hline(3));
%plotting to image
figure(1); imshow(im);
x = 1:size(im,2); y = -(hline(1) * x + hline(3)) / hline(2);
plot(x,y,'r', 'linewidth', 3);


syms f u v;
KK = [f^-2, 0, -u/f^2; 0, f^-2, -v/f^2; -u/f^2, -v/f^2, (u/f)^2+(v/f)^2+1];
eqn1 = vp(:,1)'* KK * vp(:,2) == 0;
eqn2 = vp(:,1)'* KK * vp(:,3) == 0;
eqn3 = vp(:,2)'* KK * vp(:,3) == 0;

disp('Solving equations...');
solutions = solve(eqn1, eqn2, eqn3, f, u, v);
f = eval(vpa(solutions.f(1))); f = abs(f);
u0 = eval(vpa(solutions.u(1)));
v0 = eval(vpa(solutions.v(1)));
fprintf(1,'Focal Length = %.2f, optical center at (%.2f, %.2f)\n', f, u0, v0);

%--------------------------------------------------%
%C. Solving rotation matrix
%--------------------------------------------------%
R = zeros(3,3);
K = [f, 0, u0; 0, f, v0; 0, 0, 1];
R(:,1) = K \ vp(:,1);
R(:,2) = K \ vp(:,2);
R(:,3) = - K \ vp(:,3);
% Normalization since R*R' = I
R(:,1) = R(:,1) / norm(R(:,1));
R(:,2) = R(:,2) / norm(R(:,2));
R(:,3) = R(:,3) / norm(R(:,3));
disp('Rotation Matrix R = ');
R
disp('Done');