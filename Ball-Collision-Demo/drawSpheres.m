function [] = drawSpheres(new_spheres,BC,color)
% [Chris(SIYUAN),Li, 004923970]
%This drawSpheres function takes in all the parameters from the new_spheres
%matrix and plot the current iteration of the distribution of all the
%spheres at that specific time.

%Assign Boundaries
lw = BC(1); rw = BC(2); bw = BC(3); tw = BC(4);

%Count new_spheres size
new = size(new_spheres);
ns = new(1);
p = zeros(ns,2);
for i = 1 : ns
    p(i,1) = new_spheres(i,2);
    p(i,2) = new_spheres(i,3);
end
x = p(:,1); y = p(:,2); %get the coordinates;
hold on
for i = 1 : ns %draw each circle;
    r = new_spheres(i,1);
    theta = 0:pi/100:2*pi;
    x_circle = r * cos(theta) + x(i);
    y_circle = r * sin(theta) + y(i);
    plot(x_circle, y_circle);
    fill(x_circle, y_circle, color)
end
xlim([lw rw]);
ylim([bw tw]);
set(gca,'xtick',lw:(rw-lw)/10:rw)
set(gca,'ytick',bw:(rw-lw)/10:tw)
axis square
grid on
axis([lw rw bw tw])
xlabel('BOUNDARY','FontSize',14)
ylabel('BOUNDARY','FontSize',14)
title('TEST FIELD','FontSize',14)
set(gca,'FontSize',14) % Axis fontsize
hold off
end