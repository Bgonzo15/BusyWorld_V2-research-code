function craft=AddAnAircraft(NC,craft,vehicle,NN,world,k)
%{
Generate initial conditions for a new aircraft
%}

craft(NC).x=zeros(6,world.KMAX);
craft(NC).u=zeros(3,world.KMAX);
craft(NC).xdot=zeros(6,world.KMAX);
craft(NC).r_min=NaN*zeros(1,world.KMAX); % closest distance to nearest vehicle
craft(NC).minDistance= NaN*zeros(1,world.KMAX); % closest distance to nearest vehicle
craft(NC).Rsafe=vehicle.RSafe(1) + (vehicle.RSafe(2) - vehicle.RSafe(1))*rand(1); % vehicle safe zone radius
craft(NC).xc=[0;0;0]; % location of safe sphere center in vehicle body frame
craft(NC).umax(1,1)=vehicle.vdot(1) + (vehicle.vdot(2) - vehicle.vdot(1))*rand(1); % vehicle max acceleration
craft(NC).umax(2,1)=vehicle.psidot(1) + (vehicle.psidot(2) - vehicle.psidot(1))*rand(1); % max turn rate
craft(NC).umax(3,1)=vehicle.gammadot(1) + (vehicle.gammadot(2) - vehicle.gammadot(1))*rand(1); % max flight path angle rate
craft(NC).mu=craft(NC).Rsafe^2*4*pi; % source strengths for radius R, uniform flow +1
craft(NC).wg=diag([1 1 1]);
craft(NC).S=blkdiag(2,1,1); % scaling of safe zone
craft(NC).b=[0;0;0];

% Now we go and generate initial and goal states for all the vehicles.

%craft= ComputeNearestNeighborsControl_copy(k,craft,NC,NN);
craft(NC).u(:,k)=[0;0;0]; % initially at straight & level, constant speed
[x0,xgoal]=GenerateInitialAndGoalPositions(world);
v0=vehicle.speed(1) + (vehicle.speed(2) - vehicle.speed(1))*rand(1); % compute initial speed
[psi0,gamma0]=ComputeInitialHeading(x0,xgoal);
craft(NC).x(:,k)=[x0;v0;psi0;gamma0];
craft(NC).xdot(:,k)=[v0*cos(psi0)*cos(gamma0);v0*sin(psi0)*cos(gamma0);-v0*sin(gamma0);0;0;0];
craft(NC).xgoal=xgoal;
%fprintf('   Start: %g %g %g\n',x0(1),x0(2),x0(3));
%fprintf('   Goal:  %g %g %g\n',xgoal(1),xgoal(2),xgoal(3));
%fprintf('   state: %g %g %g\n',v0,180/pi*psi0,180/pi*gamma0);
%fprintf('   xdot: %g %g %g\n',craft(NC).xdot(1,k),craft(NC).xdot(2,k),craft(NC).xdot(3,k));
craft(NC).ACTIVE=1; % activate this craft.
craft(NC).k0=k; % time this one became active

end

function [psi0,gamma0]=ComputeInitialHeading(x0,xgoal)
%{
Compute the initial heading of the aircraft
%}
r=xgoal-x0;

psi0=atan2(r(2),r(1));
gamma0=atan2(-r(3),norm(r(1:2)));
end

function [x0,xgoal]=GenerateInitialAndGoalPositions(world)
%{
Compute an initial position of the aircraft somewhere on the boundary of
world
first pick a starting face, then pick a finishing face that is not the same
as the starting face.
Then choose a position in each face.
faces: 1 is z=zmin; 2 is z=zmax; 3 is x=xmin; 4 is x=xmax; 5 is y=ymin; 6
is y=ymax
%}

x0=zeros(3,1);
xgoal=zeros(3,1);

FF=0.9; % use this to keep the initial and goal points away from the edges of the world.

face0=ceil(6*rand(1));
face1=face0;
while face1==face0
    face1=ceil(6*rand(1));
end

switch face0
    case 1
        x0(3)=world.zlim(1);
        x0(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        x0(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
    case 2
        x0(3)=world.zlim(2);
        x0(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        x0(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
    case 3
        x0(1)=world.xlim(1);
        x0(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
        x0(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 4
        x0(1)=world.xlim(2);
        x0(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
        x0(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 5
        x0(2)=world.ylim(1);
        x0(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        x0(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 6
        x0(2)=world.ylim(2);
        x0(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        x0(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
end

switch face1
    case 1
        xgoal(3)=world.zlim(1);
        xgoal(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        xgoal(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
    case 2
        xgoal(3)=world.zlim(2);
        xgoal(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        xgoal(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
    case 3
        xgoal(1)=world.xlim(1);
        xgoal(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
        xgoal(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 4
        xgoal(1)=world.xlim(2);
        xgoal(2)=FF*world.ylim(1) + FF*(world.ylim(2)-world.ylim(1))*rand(1);
        xgoal(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 5
        xgoal(2)=world.ylim(1);
        xgoal(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        xgoal(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
    case 6
        xgoal(2)=world.ylim(2);
        xgoal(1)=FF*world.xlim(1) + FF*(world.xlim(2)-world.xlim(1))*rand(1);
        xgoal(3)=FF*world.zlim(1) + FF*(world.zlim(2)-world.zlim(1))*rand(1);
end
end


