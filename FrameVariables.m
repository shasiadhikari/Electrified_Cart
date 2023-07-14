
%% Given are the parameters that influence the dynamics of the eCART  
clear all;
close all;
clc; 

%% Mass
Mv= 80 ;    %eCart weight (kg)
Md = 100 ;   %Driver weight (kg)
M = Mv+Md;  %Total weight (kg)


%% Air Resistance
Af = 1;     %projected surface area perpendicular to the direction of motion(m2)
Cd = 0.32;  %aerodynamic drag coefficient
D = 1.2;    % density of air in (Kg/m3)
v = 0.2;      %velocity of air (m/s)


%% Rolling Resistance and slope
u = 0.01;   %rolling resistance force constant 
theta = 0.08;  %slope of the ground
r= 0.2;     %radius of wheel in m
g =9.8;     %acceleration due to gravity (m/s^2)
Jw = 4;     %wheel moment of inertia 

%% Motor parameters

Gr = 80/11;                             %Gear ratio
Km = 0.310;                        % torgue constant 
Jm= 4.5*10^-4;                         % rotor (actuator and gear) inertias 
L=0.485*10^-3;                           % armature inductance
R=0.38;                                 % armature resistance 
T_stall = 38;                    % stall torgue 
Omega_nl=5363*2*pi/60;                  
Current_nl=5.11;                     % armature current 
Kb=(T_stall*R)/(Omega_nl*Km);
Bm=(Km*Current_nl)/Omega_nl;
