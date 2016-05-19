
close all,clear all , clc;


%%
for i=1:21

%% Input image
RGB = imread('ROOMGA.png');
figure('Position',[600 0 600 1000],'color','k');
image(RGB);
hold on;
axis off
axis image

%% Convert to binary image matrix and inverse matrix values

%Ixg = 250; yg = 150 = rgb2gray(RGB);
binaryImage = im2bw(RGB, 0.3);
binaryImage = 1-binaryImage;

%%
xyzO=[62, 65, 1.5, 1 ;
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

image_size=[20,15];
area_projected=image_size.* xyzO(i,3);

xs = xyzO(i,1)+(area_projected(1)/2) ; ys = round(150-xyzO(i,2)-(area_projected(2)/2));

% Goal Pose
xg = xyzO(i+1,1)+(area_projected(1)/2) ; yg = round(150-xyzO(i+1,2)-(area_projected(2)/2));
 
if(xs==0)
    xs=1;
end
if(ys==0)
    ys=1;
end
if(xg==0)
    xg=1;
end
if(yg==0)
    yg=1;
end

%% Attractive potential

sz  = size(binaryImage);
for i = 1:sz(1)
    for j = 1:sz(2)
       rg = sqrt((i-yg)^2 +(j - xg)^2);
       U_att(i,j) = 0.5*rg^2;
    end
end
contour(U_att,20);

set(gcf, 'InvertHardCopy', 'off');
% Pose Matrix
pose = zeros(sz(1),sz(2));
pose(ys,xs) = 1;
pose(yg,xg) = 1;
spy(pose,'*r');
title({'Attractive Potential Field';...
   'Circles Center Corespondents to the Goal Position '},'color','w');
hold off;

%% Repulsive potential

Di = 35; % Distance of influence of the object
% Compute for every pixel the distance to the nearest obstacle(non zero
% elements which after inversion corespondent to the obstacles)
[D,IDX] = bwdist(binaryImage);
for i = 1:sz(1)
    for j = 1:sz(2)
        rd = D(i,j);
        if (rd <= Di)
            U_rep(i,j)  = 0.5*(rd - Di)^2;
        else
            U_rep(i,j)  = 0;
        end
    end
end
figure('Position',[600 0 600 1000],'color','k');
hold on;
colormap jet;
contourf(U_rep,5);
spy(pose,'*r');
axis off
axis image
set(gcf, 'InvertHardCopy', 'off');
title('Repulsive Potential Field','color','w');
hold off;


%% Summary Potential Field

k_s  = 2; % Scaling factor
U_sum = U_rep/max(U_rep(:))+k_s*U_att/max(U_att(:));
figure('Position',[600 0 600 1000],'color','k');
hold on;
contourf(U_sum,15);
spy(pose,'*r');
axis off
axis image
title('Summary Potential Field','color','w');
set(gcf, 'InvertHardCopy', 'off');
hold off;

% %% Potential Field Path Planning
% 
% figure('Position',[600 0 600 1000],'color','k');
% hold on;
% contourf(U_sum,15);
% spy(pose,'*r');
% axis off
% axis image
% title('Potential Field Path Planning,*Note the Potential Trap','color','w');
% x = xs; y = ys;
% y=200;
% last = U_sum(y,x); % y-1,x-1
% aaa=(x ~= xg)
% bbb=(y ~= yg)
% while (aaa(1)||bbb(1))  % (aaa(1)&&aaa(2))||(bbb(1)&&bbb(2)
%     
%     if (x<2) % y-1 || x-1 || y || x ==0
%         x=2;
%     end
%     
%     if ( y<2)
%         y=2;
%     end
%      
%     if (  x >=size(U_sum,1))
%         x=size(U_sum,1)-2; % size(U_sum,1)
%     end
%     
%     if ( y >=size(U_sum,2))
%         y=size(U_sum,2)-2;
%     end
%     
% dis=[ U_sum(y-1,x-1), U_sum(y-1,x),U_sum(y-1,x+1);
%         U_sum(y,x-1), U_sum(y,x) ,U_sum(y,x+1);
%         U_sum(y+1,x-1), U_sum(y+1,x),U_sum(y+1,x+1)];
%      m = min(dis(:));
%      [r,c] = find(dis == m);
%      %r=3,c=2;   
%      U_sum(y,x) = inf;
%    y = y-2+r(1);
%    x = x-2+c(1);
%    pose(y,x) = 1;
% end
%   
% spy(pose,'.r');
% set(gcf, 'InvertHardCopy', 'off');
% hold off;

end
