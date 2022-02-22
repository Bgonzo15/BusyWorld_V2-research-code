function craft=ComputeNearestNeighborsControl(k,craft,NCRAFT,NN)

craft = GetNearestNeighbor(k,craft,NCRAFT,NN);%gets nearest neighbor index
craft = ComputeMinimumDistance(k,craft,NCRAFT);

for nc=1:NCRAFT
    if craft(nc).ACTIVE
    goalvec = craft(nc).xgoal(1:3)-craft(nc).x(1:3,k);
    v_goal = craft(nc).x(4,k)*goalvec/norm(goalvec); % desired velocity is in direction of goal.
    v_ind = ComputeInducedVelocity(k,nc,craft);%this loops over index
    v_des = v_goal+ v_ind; %this is the goal seeking potential %loops over index
    craft(nc).u(:,k) = ControlFromVelocityField(v_des,craft(nc).x(:,k),craft(nc).umax);%this is where we calculate controls
    end 
end

end

%%FUNCTIONS BELOW:
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
    if and(n~=nc,craft(nc).ACTIVE)
        r_g = x-craft(index(n)).x(1:3,k); % relative position in inertial frame
        if r_g == zeros(3,1)
            r_g = 0.01*ones(3,1);
        end
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
function v=ComputeVelocityFromSphericalSafeZone(r,xc,Mu)
%{
Computes the induced velocity at point r caused by source (for x,
y, and z relative velocities)
%}

dx = (Mu*(2*r(1) - 2*xc(1)))/(8*pi*((r(1) - xc(1))^2 + (r(2) - xc(2))^2 + (r(3) - xc(3))^2)^(3/2));
dy = (Mu*(2*r(2) - 2*xc(2)))/(8*pi*((r(1) - xc(1))^2 + (r(2) - xc(2))^2 + (r(3) - xc(3))^2)^(3/2));
dz = (Mu*(2*r(3) - 2*xc(3)))/(8*pi*((r(1) - xc(1))^2 + (r(2) - xc(2))^2 + (r(3) - xc(3))^2)^(3/2));

v =[ dx(1,1); dy(1,1);dz(1,1)];
end
function u=ControlFromVelocityField(v_des,x_current,umax)
%{
Compute turn rate and flight path angle rate from desired and current
velocity.
%}

u=zeros(3,1);

% compute current velocity vector
v_current=ComputeVelocityFromState(x_current);

cc=v_current(1)*v_des(2) - v_current(2)*v_des(1);
dpsi=asin(cc/(norm(v_current(1:2))*norm(v_des(1:2))));
psidot=interp1(90*pi/180*[-1 1],umax(2)*[-1 1],dpsi);

g1=acos(v_des(3)/norm(v_des));
g2=acos(v_current(3)/norm(v_current));
dgamma=g1-g2;
gammadot=interp1(90*pi/180*[-1 1],umax(3)*[-1 1],dgamma);

u(1)=0;
u(2)=psidot;
u(3)=gammadot;

if and(isnan(u(2)),isnan(u(3)))
    u(2) = 0.1;
    u(3) = 0.1;
end 

end
function T=ComputeRotation(psi,gamma)

cpsi=cos(psi); spsi=sin(psi);
cgam=cos(gamma); sgam=sin(gamma);

T_psi=[cpsi spsi 0; -spsi cpsi 0; 0 0 1];
T_gam=[cgam 0 -sgam; 0 1 0 ; sgam 0 cgam];

T=T_psi*T_gam;

end
function v=ComputeVelocityFromState(x_current)
%{
Compute vehicle's current velocity in the global frame from its state (v,
psi, gamma). We haev to do this because xdot isn't computed until we
integrate kinematics later...
%}

v0=x_current(4);
cpsi=cos(x_current(5));
spsi=sin(x_current(5));
cgam=cos(x_current(6));
sgam=sin(x_current(6));
v(1,1)=v0*cpsi*cgam;
v(2,1)=v0*spsi*cgam;
v(3,1)=-v0*sgam;

end

function [craft] =ComputeMinimumDistance(k,craft,NCRAFT)
%{
compute distance to nearest neighbor
%}

for nc=1:NCRAFT
    if craft(nc).ACTIVE % only do computation for active aircraft
        rmin=9999;
        for n=1:NCRAFT
            if and(n~=nc,craft(n).ACTIVE)
                r=norm(craft(n).x(1:3,k)-craft(nc).x(1:3,k));
                if r<rmin
                    rmin=r;
                end
            end
        end
       craft(nc).r_min(k)=rmin;
    end
end

end

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
