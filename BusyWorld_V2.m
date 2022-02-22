%{
BusyWorld:
    -3D simulation of a busy airspace. Meant to test obstacle avoidance based on
a source.
    -This will permit vehicles with different properties... performance, avoid
radii.
    -For now, the avoid "bubble" is a sphere centered on the vehicle.
    -Vehicles are all kinematic with inputs u = [speed psidot gammadot]'
%}

clc
clearvars
close all

NCRAFT=50;
NN = 6;

%% Define range of allowable vehicle parameters. 
% All are defined as [MIN MAX]

vehicle.speed    = [10 30];           % speed range
vehicle.vdot     = [0 0];             % acceleration range
vehicle.psidot   = deg2rad([10 45]);  % turn rate range
vehicle.gammadot = deg2rad([10 45]);  % flight path angle rate range
vehicle.RSafe    = [50 50];           % min max safe radius

%% Define some world/simulation parameters

world.NMAX=50;          % number of active vehicles at a given time
world.tmax=3600;        % simulation time (s)
world.dt= 0.1;          % time step
world.xlim=[-500 500];
world.ylim=[-500 500];
world.zlim=[-500 0];    % NED coordinate frame!!
world.KMAX=1000;        %world.tmax/world.dt;
world.t=world.dt*(0:world.KMAX-1);
world.N_ACTIVE=zeros(size(world.t));

%% Simulation
craft=BusyWorld_Initialize(0,vehicle,world,NCRAFT);
tic
[craft,world]=BusyWorld_Run(1,world,vehicle,craft,NCRAFT,NN);
toc

NCRAFT=length(craft);
figure(2)
plot(world.t,craft(1).minDistance)
xlabel("Time [s]");
ylabel("Seperation [m]");
title("Distance to Nearest n Neighbors [NN = 6]");
hold on
for n=2:NCRAFT
    if and(craft(n).minDistance(:,200) < 700, craft(n).minDistance(:,500) < 700) 
    plot(world.t,craft(n).minDistance)
    end 
end 
