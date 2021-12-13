function [new_spheres] = absorption(spheres,p,obj1,obj2,min_time,BC)
% [Chris(SIYUAN),Li, 004923970]
% This absorption function takes in the spheres matrix, density p, colliding time,
% and the colliding objects, obj1&obj2. Then, by performing inelastic
% collision, the two colliding spheres are merged and their new position,
% speed and radius data are returned along with other spheres in the new
% matrix named new_spheres.
i = 1; j = 1;
count = size(spheres);
ns = count(1);
new_spheres = zeros(ns-1,5); %create a new matrix that is one object fewer than the original one since two spheres collided into 1.
while i <= ns %copy every spheres besides the colliding two;
    if i ~= obj1 && i ~= obj2
        new_spheres(j,:) = spheres(i,:);
    else
        j = j - 1;
    end
    i = i + 1;
    j = j + 1;
end
%New radius after absorption
r1 = spheres(obj1,1); r2 = spheres(obj2,1);
rn = (r1^3+r2^3)^(1/3);
%Mass and position
m1 = p*(4/3)*pi.*spheres(obj1,1)^3; m2 = p*(4/3)*pi*spheres(obj2,1)^3;
x1 = spheres(obj1,2)+min_time*spheres(obj1,4);y1 = spheres(obj1,3)+min_time*spheres(obj1,5);
x2 = spheres(obj2,2)+min_time*spheres(obj2,4);y2 = spheres(obj2,3)+min_time*spheres(obj2,5);
%New position
xn = (m1*x1+m2*x2)/(m1+m2); yn = (m1*y1+m2*y2)/(m1+m2);
%Original velocity
v1xi = spheres(obj1,4); v1yi = spheres(obj1,5);
v2xi = spheres(obj2,4); v2yi = spheres(obj2,5);
%New velocity
nvx = (m1*v1xi+m2*v2xi)/(m1+m2); nvy = (m1*v1yi+m2*v2yi)/(m1+m2);
%Generate new sphere
new_spheres(ns-1,1)=rn;
new_spheres(ns-1,2)=xn;
new_spheres(ns-1,3)=yn;
new_spheres(ns-1,4)=nvx;
new_spheres(ns-1,5)=nvy;
%special case---near wall merging
if abs(BC(1)-xn)<rn
    %Left wall
    new_spheres(ns-1,2)=BC(1)+rn;
elseif abs(BC(2)-xn)<rn
    %Right wall
    new_spheres(ns-1,2)=BC(2)-rn;
elseif abs(BC(3)-yn)<rn
    %Bottom wall
    new_spheres(ns-1,3)=BC(3)+rn;
elseif abs(BC(4)-yn)<rn
    %Top wall
    new_spheres(ns-1,3)=BC(4)-rn;
end
end


