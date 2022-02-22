function [craft] =GetNearestNeighbor(k,craft,NCRAFT,NN)
%{
    -This function computes the distance between one aircraft to all other
    aircrafts in BusyWorld
    -It then takes those distances and sorts them from nearest to farthest
    neighbors
    -Returns craft that holds a field called
    craft(nc).minDistance(k) = [pulls the min distance from one aircraft
    and all other aircrafts]
%}
for nc=1:NCRAFT %this loop goes through each NCRAFT in the world and gets its nearest neighbors ID and distances
    if craft(nc).ACTIVE % only do computation for active aircraft
        rmin=9999;
        for n=1:NCRAFT
            if and(n~=nc,craft(n).ACTIVE)
                %sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2))
                r=norm(craft(n).x(1:3,k)-craft(nc).x(1:3,k));%distance from one aircraft and all other aircrafts
                if r>rmin
                    r=rmin;
                end
                craft(nc).distanceArray(n,:) = [craft(n).ID r];
            else
                craft(nc).distanceArray(n,:) = [ 0 1e3]; %test this  
            end
        end
        [B,I] = sort(craft(nc).distanceArray);
        newlySorted = [I(:,2) B(:,2)];
        craft(nc).index = newlySorted(1:NN,:);
        craft(nc).minDistance(k) = newlySorted(1,2); %this pulls the min distance from the list of closest neighbora to the aircraft of focus
    end
end
end

%{
function v_ind=ComputeInducedVelocity(k,nc,craft)
%{
Compute velocity induced from other vehicles.
%}

v_ind=zeros(3,1);
t_lookahead=0;
v_nc=ComputeVelocityFromState(craft(nc).x(:,k));
x=craft(nc).x(1:3,k)+t_lookahead*v_nc; 
index = craft(nc).index;
for n=1:length(index)
    if and(n~=nc,craft(n).ACTIVE)
        r_g = x-craft(index(n)).x(1:3,k); % relative position in inertial frame
        v_n = ComputeVelocityFromState(craft(index(n)).x(:,k));
        v_g = v_nc-v_n; % relative velocity in inertial frame
        T = ComputeRotation(craft(index((n))).x(5,k),craft(index(n)).x(6,k)); % find rotation matrix to put relative position in craft "n" frame
        r_b = T*r_g; % relative position in craft "n" frame
        vrel_b = T*v_g; % relative velocity in craft "n" frame
        Mu = craft(n).mu.*norm(vrel_b);
        v_b = ComputeVelocityFromSphericalSafeZone(r_b,craft(n).xc,Mu);
        v_g = T'*v_b; % convert to inertial frame
        %v_ind = v_ind+v_g;
       v_ind(:,1) = v_ind + v_g;
    end 
     
end
end
%}