function [craft,world]=BusyWorld_Run(PLOTFLAG,world,vehicle,craft,NCRAFT,NN)
k=1;

if PLOTFLAG
    %Open up a figure for plotting
    figure('Name','Busy World','Units','pixels','Position',[50 50 650 590])
    %{
    R=norm(craft(1).x(1:3));
    ax=subplot(1,1,1);
    [X,Y,Z]=sphere(20);
    surf(ax,R*X,R*Y,R*Z,'FaceColor','none','EdgeColor',[1 1 1])
    axis equal
    %}
    plot3(0,0,0,'b.')
    hold on
    for n=1:NCRAFT
        xx=[craft(n).x(1,k) craft(n).xgoal(1)];
        yy=[craft(n).x(2,k) craft(n).xgoal(2)];
        zz=[craft(n).x(3,k) craft(n).xgoal(3)];
        plot3(xx(1),yy(1),zz(1),'go')
        plot3(xx,yy,zz,'b:')
        plot3(xx(2),yy(2),zz(2),'ro')
        %DrawCraft(ax,craft(n).x,craft(n).Rsafe,craft(n).S,craft(n).b);
    end
    set(gca,'XLim',world.xlim,'YLim',world.ylim','ZLim',world.zlim)
    set(gca,'ZDir','reverse')
    drawnow;
    xlabel("North", 'FontSize',12);
    ylabel("East", 'FontSize',12);
    zlabel("Down", 'FontSize',12);
    title("Dense Airspace Simulation",'FontSize',12);
    
end
hold on

while k<=world.KMAX
    
    craft = ComputeNearestNeighborsControl(k,craft,NCRAFT,NN);
    
    % now integrate kinematics
    for nn=1:NCRAFT
        world.N_ACTIVE(k)=world.N_ACTIVE(k)+craft(nn).ACTIVE; % count active aircraft
        if craft(nn).ACTIVE
            [craft(nn).x(:,k+1),craft(nn).xdot(:,k)]=IntegrateKinematics(craft(nn).x(:,k),craft(nn).u(:,k),world.dt);
            dgoal=craft(nn).xgoal - craft(nn).x(1:3,k+1);
            rgoal=sqrt(dgoal'*craft(nn).wg*dgoal);
            if rgoal<craft(nn).Rsafe
                fprintf('Craft %d reached goal at time step %d.\n',nn,k+1)
                craft(nn).ACTIVE=0; % craft has reached goal... deactivate it
            end
        end
    end
    
    
    DN=(world.NMAX-world.N_ACTIVE(k))/world.NMAX;
    p_newcraft=rand(1);
    if p_newcraft<DN
        NCRAFT=NCRAFT+1;
        fprintf('Adding an aircraft at k=%d; ... there are now %d aircraft.\n',k+1,NCRAFT)
        craft=AddAnAircraft(NCRAFT,craft,vehicle,NN,world,k+1);
    end
    %}
    if mod(k,10)==0
        cla
        for nn=1:NCRAFT
            if craft(nn).ACTIVE
                k0=craft(nn).k0;
                xx=[craft(nn).x(1,k0) craft(nn).xgoal(1)];
                yy=[craft(nn).x(2,k0) craft(nn).xgoal(2)];
                zz=[craft(nn).x(3,k0) craft(nn).xgoal(3)];
                plot3(xx(1),yy(1),zz(1),'go')
                plot3(xx,yy,zz,'b:')
                plot3(xx(2),yy(2),zz(2),'go')
                
                if craft(nn).r_min(k)<1*craft(nn).Rsafe
                    plot3(craft(nn).x(1,k),craft(nn).x(2,k),craft(nn).x(3,k),'ro')
                else
                    plot3(craft(nn).x(1,k),craft(nn).x(2,k),craft(nn).x(3,k),'bo')
                end
            end
        end
        drawnow
    end
    k=k+1; 
end

%plotting active aircrafts at each time step

end

function [x,xdot]=IntegrateKinematics(x0,u,dt)
%{
x0 = [x y z va psi gamma]'
u=[vadot psidot gammadot]'
%}

xdot=Compute_xdot(x0,u);
x=x0+xdot*dt;
end
function xdot=Compute_xdot(x0,u)

cpsi=cos(x0(5));
spsi=sin(x0(5));
cgam=cos(x0(6));
sgam=sin(x0(6));

xdot(1,1)=x0(4)*cpsi*cgam;
xdot(2,1)=x0(4)*spsi*cgam;
xdot(3,1)=-x0(4)*sgam;
xdot(4,1)=u(1);
xdot(5,1)=u(2);
xdot(6,1)=u(3);
end
function PlotSituation(ax,NCRAFT,craft)
%{

%}

cla(ax);
for n=1:NCRAFT
    plot3(craft(n).x(1),craft(n).x(2),craft(n).x(3),'go');
    plot3(craft(n).xg(1),craft(n).xg(2),craft(n).xg(3),'ro');
    DrawCraft(ax,craft(n).x,craft(n).Rsafe,craft(n).S,craft(n).b);
end

drawnow;

end
function DrawCraft(ax,x,R,S,b)
% plot the craft's ellipsoid...
NS=20; % number of points to define sphere
[Ze,Ye,Xe]=sphere(NS);

Tn=ComputeRotation(x(5),x(6)); % inertial to obstacle n frame
Xn=R*S(1,1)*Xe+b(1);
Yn=R*S(2,2)*Ye+b(2);
Zn=R*S(3,3)*Ze+b(3);
for i=1:NS+1
    for j=1:NS+1
        xx=[Xn(i,j);Yn(i,j);Zn(i,j)];
        xx=Tn'*xx;
        Xn(i,j)=xx(1)+x(1);
        Yn(i,j)=xx(2)+x(2);
        Zn(i,j)=xx(3)+x(3);
    end
end
surf(ax,Xn,Yn,Zn,'FaceColor',[0 0 1],'FaceAlpha',0.2)
end
function T=ComputeRotation(psi,gamma)
%{
Calculates rotation matrix from inertial to body frame so that
x_b = T*x_i

Uses NED frame.

Inputs:
psi     heading (positive around +z)
gamma   flight path angle (positive around new +y)

Outputs:
T       direction cosine matrix

%}

cpsi=cos(psi); spsi=sin(psi);
cgam=cos(gamma); sgam=sin(gamma);

T_psi=[cpsi spsi 0; -spsi cpsi 0; 0 0 1];
T_gam=[cgam 0 -sgam; 0 1 0 ; sgam 0 cgam];

T=T_gam*T_psi;
end
