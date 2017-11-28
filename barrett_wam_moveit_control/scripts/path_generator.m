clear; close all; clc;

type='ellipse';
% type='track';
% type='figure_eight';

path='./trajectories';

% Start at base-middle of ellipse

if strcmp(type,'ellipse') 
 
 n=100;  %num. points   
    
 x1=-1.0; y1=0;  %major-axis vertices
 x2= 1.0; y2=0;
 e=0.8;  %eccentricity (0<e<1)
 
 a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
 b = a*sqrt(1-e^2);
 t = linspace(0,2*pi,n);
 X = a*cos(t);
 Y = b*sin(t);
 w = atan2(y2-y1,x2-x1);
 x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
 y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
 
 plot(x,y,'r.')
 axis equal
 
 M=[x' y'];
 
 file=sprintf('cart_path_ellipse_n_%d.csv',n);
 
end


csvwrite(fullfile(path,file),M);
