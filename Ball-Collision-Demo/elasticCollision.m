function [v1xf,v1yf,v2xf,v2yf] = elasticCollision(spheres,p,obj1,obj2,type,min_time)
% [Chris(SIYUAN),Li, 004923970]
% This elasticCollision function takes in the spheres matrix and all the
% basic parameters and calculate the speed of the balls after collision.
% Then, it will return the speed appeared after collision.

if type == 1  
    %Left wall
    v1xf = -(spheres(obj1,4)); v1yf = spheres(obj1,5);
    v2xf = 0; v2yf = 0;
elseif type == 2
    %Right wall
    v1xf = -(spheres(obj1,4)); v1yf = spheres(obj1,5);
    v2xf = 0; v2yf = 0;
elseif type == 3
    %Bottom wall
    v1xf = spheres(obj1,4); v1yf = -(spheres(obj1,5));
    v2xf = 0; v2yf = 0;
elseif type == 4
    %Top wall
    v1xf = spheres(obj1,4); v1yf = -(spheres(obj1,5));
    v2xf = 0; v2yf = 0;
elseif type == 0
    %Ball collision
    m1 = p*(4/3)*pi.*spheres(obj1,1)^3; m2 = p*(4/3)*pi*spheres(obj2,1)^3;
    x1 = spheres(obj1,2)+min_time*spheres(obj1,4);y1 = spheres(obj1,3)+min_time*spheres(obj1,5);
    x2 = spheres(obj2,2)+min_time*spheres(obj2,4);y2 = spheres(obj2,3)+min_time*spheres(obj2,5);
    a = atan2((y2-y1),(x2-x1));
    %original velocity
    v1xi = spheres(obj1,4); v1yi = spheres(obj1,5);
    v2xi = spheres(obj2,4); v2yi = spheres(obj2,5);
    %transferred velocity
    v1xi_ = v1xi*cos(a) + v1yi*sin(a); v1yi_ = v1yi*cos(a) - v1xi*sin(a);
    v2xi_ = v2xi*cos(a) + v2yi*sin(a); v2yi_ = v2yi*cos(a) - v2xi*sin(a);
    %velocity after collision in transferred frame
    v1xf_ = ((m1-m2)*v1xi_+2*m2*v2xi_)/(m1+m2);v2xf_ = ((m2-m1)*v2xi_+2*m1*v1xi_)/(m1+m2);
    %velocity after collision in original frame
    v1xf = v1xf_*cos(a)-v1yi_*sin(a); v1yf = v1yi_*cos(a)+v1xf_*sin(a);
    v2xf = v2xf_*cos(a)-v2yi_*sin(a); v2yf = v2yi_*cos(a)+v2xf_*sin(a);
end