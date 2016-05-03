%% Curve Smooth
% http://fr.mathworks.com/matlabcentral/answers/157087-interpolation-with-interp1-to-show-the-path-of-a-robot

close all , clear all , clc;
%% the path unsorted
t=[62, 65, 1.5, 1  ;
 86, 73, 1.5, 1  ;
 35, 97, 1.5, 0  ;
 21, 18, 1.5, 1  ;
 117, 107, 1.5, 0;
 119, 5, 1.5, 0	 ;
 34, 76, 1.5, 0	 ;
 110, 112, 1.5, 1;
 109, 85, 1.5, 0 ;
 1, 88, 1.5, 1   ;
 81, 31, 1.5, 0  ;
 0, 21, 1.5, 1   ;
 87, 53, 1.5, 0	 ;
 33, 55, 1.5, 0	 ;
 117, 54, 1.5, 1 ;
 3, 68, 1.5, 0   ;
 41, 28, 1.5, 1  ;
 62, 88, 1.5, 1  ;
 108, 0, 1.5, 1  ;
 3, 47, 1.5, 0   ;
 112, 27, 1.5, 1 ;
 15, 90, 1.5, 1];


%% path sorted from a mat
t=load('path_Outdoor_GPS_70Pts.mat');

t=t.areal;

%% interpolation part
spacea=linspace(0,size(t,1),size(t,1));
x = t(:,1);
y = t(:,2);
% calculate spline for way points
tq = 0:0.1:size(t,1);
xq = interp1(spacea,x,tq,'spline');
yq = interp1(spacea,y,tq,'spline');

%% show result by ploting
plot(xq,yq)

