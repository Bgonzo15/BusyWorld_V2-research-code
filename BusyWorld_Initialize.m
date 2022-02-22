function craft=BusyWorld_Initialize(PLOTFLAG,vehicle,world,NCRAFT)
%{
Generate the initial set of aircraft in BusyWorld
craft.t          vector of time...
craft.x
craft.u
craft.xdot
craft.RSafe
craft.mu
%}

craft=struct;
% Initialization of Matrices
for n=1:NCRAFT
    craft(n).x= zeros(6,world.KMAX);
    craft(n).u= zeros(3,world.KMAX);
    craft(n).xdot= zeros(6,world.KMAX);
    craft(n).r_min= NaN*zeros(1,world.KMAX); % closest distance to nearest vehicle
    craft(n).minDistance= NaN*zeros(1,world.KMAX); % closest distance to nearest vehicle
    craft(n).Rsafe=vehicle.RSafe(1) + (vehicle.RSafe(2) - vehicle.RSafe(1))*rand(1); % vehicle safe zone radius
    craft(n).xc=[0;0;0]; % location of safe sphere center in vehicle body frame
    craft(n).umax(1,1)=vehicle.vdot(1) + (vehicle.vdot(2) - vehicle.vdot(1))*rand(1); % vehicle max acceleration
    craft(n).umax(2,1)=vehicle.psidot(1) + (vehicle.psidot(2) - vehicle.psidot(1))*rand(1); % max turn rate
    craft(n).umax(3,1)=vehicle.gammadot(1) + (vehicle.gammadot(2) - vehicle.gammadot(1))*rand(1); % max flight path angle rate
    craft(n).mu=craft(n).Rsafe^2*4*pi; % source strengths for radius R, uniform flow +1
    craft(n).wg=diag([1 1 1]);
    craft(n).S=blkdiag(2,1,1); % scaling of safe zone
    craft(n).b=[0;0;0];
end

% Generate initial and goal states for all vehicles
dist=1000*ones(1,NCRAFT);

for n=1:NCRAFT
    %fprintf('Generating initial state for craft %d of %d\n',n,NCRAFT)
    % craft(n).u(:,1)=[0;0;0]; % Initially at straight & level, constant speed
    VALIDSTART=0;
    while VALIDSTART==0
        [x0,xgoal]=GenerateInitialAndGoalPositions(world);
        for nn=1:n-1
            dist(nn)=norm(x0-craft(nn).x(1:3,1)); % Check to make sure that this initial point is not close to anyone
        end
        if min(dist)>craft(n).Rsafe
            VALIDSTART=1;
            dist= 1e10*ones(1,NCRAFT);
        end
    end
    
    %Getting the ID number for a craft
    for nc = 1:NCRAFT
        craft(nc).ID = nc;
    end
    
    v0=vehicle.speed(1) + (vehicle.speed(2) - vehicle.speed(1))*rand(1); % Compute initial speed
    [psi0,gamma0]=ComputeInitialHeading(x0,xgoal);
    craft(n).x(:,1)=[x0;v0;psi0;gamma0];
    craft(n).xdot(:,1)=[v0*cos(psi0)*cos(gamma0);v0*sin(psi0)*cos(gamma0);-v0*sin(gamma0);0;0;0];
    craft(n).xgoal=xgoal;
    %fprintf('   Start: %g %g %g\n',x0(1),x0(2),x0(3));
    %fprintf('   Goal:  %g %g %g\n',xgoal(1),xgoal(2),xgoal(3));
    %fprintf('   state: %g %g %g\n',v0,180/pi*psi0,180/pi*gamma0);
    craft(n).ACTIVE=1; % activate this craft.
    craft(n).k0=1; % index this one became active
end


if PLOTFLAG
    figure('Name','BusyWorld start condition')
    plot3(0,0,0,'b.')
    hold on
    for n=1:NCRAFT
        xx=[craft(n).x(1,1) craft(n).xgoal(1)];
        yy=[craft(n).x(2,1) craft(n).xgoal(2)];
        zz=[craft(n).x(3,1) craft(n).xgoal(3)];
        plot3(xx(1),yy(1),zz(1),'go')
        plot3(xx,yy,zz,'b:')
        plot3(xx(2),yy(2),zz(2),'ro')
    end
    set(gca,'XLim',world.xlim,'YLim',world.ylim','ZLim',world.zlim)
    set(gca,'ZDir','reverse')
end
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

