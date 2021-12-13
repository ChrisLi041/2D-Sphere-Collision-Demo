function [new_spheres,new_time,flag] = fieldEvolution(spheres,BC,dt,AR,p,time)
% [Chris(SIYUAN),Li, 004923970]
% This fieldEvolution function takes in all the basic parameters, including
% the original spheres matrix, ns, Boundary conditions, the starting and
% final time, dt, absorption ratio(AR), and the density(p). And it will
% detect collision and return the sphere matrix with the updated
% parameters.

%% Spheres Evolution
new_spheres = spheres;
flag = 0; %flag of collision (0=no collision; 1=collision)
index = 1;
collision = zeros; %matrix that stores the colliding objects' information and the shorted time;
new = size(spheres);
ns = new(1); %count the size every time a new_spheres matrix is passed in;
%% Update address
for i = 1 : 1 : ns
    cx = spheres(i,2); cy = spheres(i,3);
    cvx = spheres(i,4); cvy = spheres(i,5);
    nx = cx + cvx*dt;
    ny = cy + cvy*dt;
    new_spheres(i,2)=nx; new_spheres(i,3)=ny;
end
%% Check sphere collision
for current = 1 : 1 : ns
    for before = 1 : 1 : current-1
        xb = new_spheres(before,2); yb = new_spheres(before,3);
        xc = new_spheres(current,2); yc = new_spheres(current,3);
        diff = sqrt((xc-xb)^2+(yc-yb)^2);
        if diff < (new_spheres(before,1) + new_spheres(current,1))
            %spheres collide
            flag = 1;
            v1 = sqrt((spheres(before,4))^2+(spheres(before,5))^2);
            v2 = sqrt((spheres(current,4))^2+(spheres(current,5))^2);
            x1 = spheres(before,2); y1 = spheres(before,3);
            x2 = spheres(current,2); y2 = spheres(current,3);
            dis = sqrt((x1-x2)^2+(y1-y2)^2)-spheres(before,1)-spheres(current,1);
            t = dis/(v1+v2);
            %store in the collision matrix;
            collision(1,index) = before;
            collision(2,index) = current;
            collision(3,index) = t;
            index = index + 1;
        end
    end
end
%% Check wall collision
for new = 1 : 1 : ns %check every ball's new position;
    if new_spheres(new,2) <= BC(1)+spheres(new,1)
        %collide with left wall
        flag = 1;
        t = (abs(spheres(new,2)-BC(1))-spheres(new,1))/abs(spheres(new,4));
        %store in the collision matrix;
        collision(1,index)=new;
        collision(2,index)=-1;
        collision(3,index)=t;
        index = index + 1;
    end
    if new_spheres(new,2) >= BC(2)-spheres(new,1)
        %collide with right wall
        flag = 1;
        t = (abs(spheres(new,2)-BC(2))-spheres(new,1))/abs(spheres(new,4));
        %store in the collision matrix;
        collision(1,index)=new;
        collision(2,index)=-2;
        collision(3,index)=t;
        index = index + 1;
    end
    if new_spheres(new,3) <= BC(3)+spheres(new,1)
        %collide with bottom wall
        flag = 1;
        t = (abs(spheres(new,3)-BC(3))-spheres(new,1))/abs(spheres(new,5));
        %store in the collision matrix;
        collision(1,index)=new;
        collision(2,index)=-3;
        collision(3,index)=t;
        index = index + 1;
    end
    if new_spheres(new,3) >= BC(4)-spheres(new,1)
        %collide with top wall
        flag = 1;
        t = (abs(spheres(new,3)-BC(4))-spheres(new,1))/abs(spheres(new,5));
        %store in the collision matrix;
        collision(1,index)=new;
        collision(2,index)=-4;
        collision(3,index)=t;
        index = index + 1;
    end
end

if flag == 1
    [min_time,inx]=min(collision(3,:));
    new_time = time + min_time; %update the new time based on the shorted time of collision;
    if collision(2,inx) == -1
        %LEFT WALL
        type = 1;
        obj1 = collision(1,inx);
        obj2 = collision(2,inx);
        [v1xf,v1yf,~,~] = elasticCollision(spheres,p,obj1,obj2,type,min_time); %get the velocity of the sphere after bouncing off the wall;
        for i = 1 : 1 : ns  %copy the non-colliding spheres data;
            cx = spheres(i,2); cy = spheres(i,3);
            cvx = spheres(i,4); cvy = spheres(i,5);
            nx = cx + min_time*cvx; ny = cy + min_time*cvy;
            new_spheres(i,2)=nx; new_spheres(i,3)=ny;
        end
        new_spheres(obj1,4)=v1xf; new_spheres(obj1,5)=v1yf; %update velocity after collision;
    elseif collision(2,inx) == -2
        %RIGHT WALL
        type = 2;
        obj1 = collision(1,inx);
        obj2 = collision(2,inx);
        [v1xf,v1yf,~,~] = elasticCollision(spheres,p,obj1,obj2,type,min_time); %get the velocity of the sphere after bouncing off the wall;
        for i = 1 : 1 : ns %copy the non-colliding spheres data;
            cx = spheres(i,2); cy = spheres(i,3);
            cvx = spheres(i,4); cvy = spheres(i,5);
            nx = cx + min_time*cvx; ny = cy + min_time*cvy;
            new_spheres(i,2)=nx; new_spheres(i,3)=ny;
        end
        new_spheres(obj1,4)=v1xf; new_spheres(obj1,5)=v1yf;%update velocity after collision;
    elseif collision(2,inx) == -3
        %BOTTOM WALL
        type = 3;
        obj1 = collision(1,inx);
        obj2 = collision(2,inx);
        [v1xf,v1yf,~,~] = elasticCollision(spheres,p,obj1,obj2,type,min_time); %get the velocity of the sphere after bouncing off the wall;
        for i = 1 : 1 : ns %copy the non-colliding spheres data;
            cx = spheres(i,2); cy = spheres(i,3);
            cvx = spheres(i,4); cvy = spheres(i,5);
            nx = cx + min_time*cvx; ny = cy + min_time*cvy;
            new_spheres(i,2)=nx; new_spheres(i,3)=ny;
        end
        new_spheres(obj1,4)=v1xf; new_spheres(obj1,5)=v1yf;%update velocity after collision;
    elseif collision(2,inx) == -4
        %TOP WALL
        type = 4;
        obj1 = collision(1,inx);
        obj2 = collision(2,inx);
        [v1xf,v1yf,~,~] = elasticCollision(spheres,p,obj1,obj2,type,min_time); %get the velocity after the collision;
        for i = 1 : 1 : ns %update the addresses of all the spheres;
            cx = spheres(i,2); cy = spheres(i,3);
            cvx = spheres(i,4); cvy = spheres(i,5);
            nx = cx + min_time*cvx; ny = cy + min_time*cvy;
            new_spheres(i,2)=nx; new_spheres(i,3)=ny;
        end
        new_spheres(obj1,4)=v1xf; new_spheres(obj1,5)=v1yf;%update velocity after collision;
    else
        %BALL COLLISION
        type = 0;
        obj1 = collision(1,inx);
        obj2 = collision(2,inx);
        pick = rand;
        if pick>AR %decide if the collision is elastic based on the absorption ratio;
            %elastic
            [v1xf,v1yf,v2xf,v2yf] = elasticCollision(spheres,p,obj1,obj2,type,min_time); %get the velocity of the spheres after elastic colllision;
            for i = 1 : 1 : ns %update the positions of all the spheres;
                cx = spheres(i,2); cy = spheres(i,3);
                cvx = spheres(i,4); cvy = spheres(i,5);
                nx = cx + min_time*cvx; ny = cy + min_time*cvy;
                new_spheres(i,2)=nx; new_spheres(i,3)=ny;
            end
            new_spheres(obj1,4)=v1xf; new_spheres(obj1,5)=v1yf; %update the velocity for object 1;
            new_spheres(obj2,4)=v2xf; new_spheres(obj2,5)=v2yf; %update the velocity for object 1;
        else
            %inelastic
            [new_spheres] = absorption(spheres,p,obj1,obj2,min_time,BC); %perform inelastic collision;
            % imediate catch
            new = size(new_spheres);
            count_new = new(1); %count the current number of spheres after merging;
            m = 1;
            while m <= count_new %check if the new merged sphere intersect with the original spheres;
                for n = 1 : 1 : m-1 %check all the spheres;
                    xb = new_spheres(n,2); yb = new_spheres(n,3);
                    xc = new_spheres(m,2); yc = new_spheres(m,3);
                    diff = sqrt((xc-xb)^2+(yc-yb)^2);
                    if diff < (new_spheres(n,1) + new_spheres(m,1))
                        [new_spheres] = absorption(new_spheres,p,m,n,0,BC);
                        count_new = count_new - 1; %one less rows after an additional merge;
                        m = 0; %change m back to 0 and re-check if any merging takes place.
                        break
                    end
                end
                m = m + 1; %update the index;
            end
        end
    end
else
    new_time = time + dt; %no collision (flag=0) update by the original dt;
end

end