function vp = getVanishingPoint(im, x1, y1, x2, y2, x3, y3, x4, y4)
% output vanishing point, input image
figure(1), hold off, imagesc(im)
hold on 

lines = zeros(3, 0);
line_length = zeros(1,0);
centers = zeros(3, 0);



lines(:, end+1) = real(cross([x1 y1 1]', [x2 y2 1]'));
lines(:, end+1) = real(cross([x3 y3 1]', [x4 y4 1]'));
line_length(end+1) = sqrt((y2-y1)^2 + (x2-x1).^2);
line_length(end+1) = sqrt((y4-y3)^2 + (x4-x3).^2);
centers(:, end+1) = [x1+x2 y1+y2 2]/2;
centers(:, end+1) = [x3+x4 y3+y4 2]/2;

%% solve for vanishing point 
% Insert code here to compute vp (3x1 vector in homogeneous coordinates)
n = size(lines,2);
vpCandidate = zeros(3,0);
for i = 1:n-1
    for j = (i+1):n
        vpCandidate(:,end+1) = cross(lines(:,i), lines(:,j));
    end
end

vp = getVP(centers, vpCandidate);
vp = vp ./ vp(3);

%% display 
hold on
bx1 = min(1, vp(1)/vp(3))-10; bx2 = max(size(im,2), vp(1)/vp(3))+10;
by1 = min(1, vp(2)/vp(3))-10; by2 = max(size(im,1), vp(2)/vp(3))+10;
for k = 1:size(lines, 2)
    if lines(1,k)<lines(2,k)
        pt1 = real(cross([1 0 -bx1]', lines(:, k)));
        pt2 = real(cross([1 0 -bx2]', lines(:, k)));
    else
        pt1 = real(cross([0 1 -by1]', lines(:, k)));
        pt2 = real(cross([0 1 -by2]', lines(:, k)));
    end
    pt1 = pt1/pt1(3);
    pt2 = pt2/pt2(3);
    plot([pt1(1) pt2(1)], [pt1(2) pt2(2)], 'g', 'Linewidth', 1);
end
plot(vp(1)/vp(3), vp(2)/vp(3), '*r')
axis image
axis([bx1 bx2 by1 by2]); 

