function [spheres] = seedInitial(ns,vs,rs,BC)
% [Chris(SIYUAN),Li, 004923970]
% This seedInitial function will generate and return a ns × 5 array called spheres, with each row of
% array corresponding to one sphere. Inside the array, the information
% about radius, x-coordinate, y-coordinate, x-velocity and y-velocity of
% each sphere will be generated and stored.

%% Initialize the spheres matrix
spheres = zeros(ns,5);

%% Boundary Check
BCsize = size(BC); %check the boundary size;
if BCsize(1) ~= 4 %check if the size is valid;
    error('CHECK BOUNDARY SIZE');
else %store all the boundary conditions;
    x_min = BC(1); x_max = BC(2); y_min = BC(3); y_max = BC(4); 
    if x_min >= x_max || y_min >= y_max %check if the limits are valid;
        error('CHECK BOUNDARY LIMIT');
    end
end

%% Radius
RSsize = size(rs); %store the size of the radius data;
if RSsize == 1 %if the radius is a constant;
    if rs <= 0 || rs >= (x_max-x_min)/2 || rs >= (y_max-y_min)/2 %check if the radius is valid;
        error('CHECK RADIUS MAGNITUDE');
    end
    for i = 1 : 1 : ns
        spheres(i,1) = rs; %fill in the spheres matrix with the radius data;
    end
elseif RSsize(1) == ns %check if the radius array is valid;
    for i = 1 : 1 : ns
        if rs(i,1) <= 0 || rs(i,1) >= (x_max-x_min)/2 || rs(i,1) >= (y_max-y_min)/2 %check if the radius data make sense;
            error('CHECK RADIUS MAGNITUDE');
        end
        spheres(i,1) = rs(i,1); %fill in the radius data;
    end
else
    error('CHECK NUMBER OF RADIUS ENTERED'); %display error message if it is neither of the two cases above;
end

%% x&y-coordinate
i = 1;
if RSsize(1) == ns
    while i <= ns %for all the spheres;
        xc = ((x_max - rs(i,1)) - (x_min + rs(i,1))).*rand + (x_min + rs(i,1)); %generate a random x position within the BC;
        yc = ((y_max - rs(i,1)) - (y_min + rs(i,1))).*rand + (y_min + rs(i,1)); %generate a random Y position within the BC;
        spheres(i,2) = xc; spheres(i,3) = yc; %fill in the x&y position to the matrix;
        for j = 1 : 1 : i-1 %check if the new generated sphere overlaps with the original ones;
            diff_x = abs(xc - spheres(j,2)); diff_y = abs(yc - spheres(j,3)); %calculate the radius difference;
            tot_diff=sqrt((diff_x)^2+(diff_y)^2)-(rs(i,1)+rs(j,1));
            if tot_diff < 0 %check if overlap;
                i = i - 1; %regenerate this sphere by subtracting the index by 1;
                break
            end
        end
        i = i + 1;
    end
else %if the radius is uniform
    while i <= ns %for all the spheres;
        xc = ((x_max - rs) - (x_min + rs)).*rand + (x_min + rs); %generate a random x position within the BC;
        yc = ((y_max - rs) - (y_min + rs)).*rand + (y_min + rs); %generate a random Y position within the BC;
        spheres(i,2) = xc; spheres(i,3) = yc; %fill in the x&y position to the matrix;
        for j = 1 : 1 : i-1 %check if the new generated sphere overlaps with the original ones;
            diff_x = abs(xc - spheres(j,2));
            diff_y = abs(yc - spheres(j,3)); %calculate the radius difference;
            tot_diff=sqrt(diff_x^2+diff_y^2)-2*rs;
            if tot_diff <= 0 %check if overlap;
                i = i - 1; %regenerate this sphere by subtracting the index by 1;
                break
            end
        end
        i = i + 1;
    end
end


%% x&y-velocity
% The x and y velocity of each sphere will be assigned using the vs value. Each
% sphere will move in a random direction, with the velocity magnitude equal to vs.
% If it is just a single term, then each sphere has this velocity magnitude. If it is a
% (ns × 1) array, then calibrate the velocity using these magnitudes one-by-one,
% respectively.
VSsize = size(vs); %store the size of the velocity data;
if VSsize == 1 %if the radius is a constant;
    if vs <= 0 || vs > 3e8 %check if the radius is valid (cannot exceed the speed of light);
        error('CHECK SPEED MAGNITUDE');
    end
    for i = 1 : 1 : ns
        vx = (2*vs).*rand + (-vs); 
        vy = (2*sqrt(vs^2-vx^2)).*rand + (-sqrt(vs^2-vx^2));
        spheres(i,4) = vx; %generate random x velocity;
        spheres(i,5) = vy; %generate random y velocity;
    end
elseif RSsize(1) == ns %check if the radius array is valid;
    for i = 1 : 1 : ns
        if vs(i,1) < 0 || vs(i,1) > 3e8 %check if the radius data make sense;
            error('CHECK SPEED MAGNITUDE');
        end
        vx = (2*vs(i,1)).*rand + (-vs(i,1)); 
        vy = (2*sqrt(vs(i,1)^2-vx^2)).*rand + (-sqrt(vs(i,1)^2-vx^2));
        spheres(i,4) = vx; %generate random x velocity;
        spheres(i,5) = vy; %generate random y velocity;
    end
else
    error('CHECK NUMBER OF SPEED DATA ENTERED'); %display error message if it is neither of the two cases above;
end
