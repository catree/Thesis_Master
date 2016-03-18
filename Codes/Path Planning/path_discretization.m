%%
close all , clear all, clc;
%% Path

% this path is given by the 
targets=[		0,  46,      3.5;
    0,  16,      3.0;
    2,  7,       3.5;
    1,  76,      3.5;
    41,  50,     3.5;
    43,  16,     3.5;
    68,  61,     3.5;
    46,  92,     3.5;
    105,  90 ,   3.5;
    101,  56 ,   3.5;
    88,  25,     3.5;
    104,  1,     3.5  ];
%%
a=[];
b=[];
c=[];

threshold_interpolation=5;
for i=1:size(targets,1)-1
    
    if (abs(targets(i,2)-targets(i+1,2))>abs(targets(i,1)-targets(i+1,1)))
        if (abs(targets(i,2)-targets(i+1,2))>threshold_interpolation)
    a=[a;linspace(targets(i,2),targets(i+1,2),threshold_interpolation)'];
    b=[b;linspace(targets(i,1),targets(i+1,1),threshold_interpolation)'];
    c=[c;targets(i,3)*ones(threshold_interpolation,1)];
        else 
    a=[a;linspace(targets(i,2),targets(i+1,2),abs(targets(i,2)-targets(i+1,2)))'];
    b=[b;linspace(targets(i,1),targets(i+1,1),abs(targets(i,2)-targets(i+1,2)))'];
    c=[c;targets(i,3)*ones(abs(targets(i,2)-targets(i+1,2)),1)];   
        end 
    scale=linspace(targets(i+1,2),targets(i,2),targets(i,2)-targets(i+1,2))';
      
    else
        if (abs(targets(i,1)-targets(i+1,1))>threshold_interpolation)
    a=[a;linspace(targets(i,2),targets(i+1,2),threshold_interpolation)'];
    b=[b;linspace(targets(i,1),targets(i+1,1),threshold_interpolation)'];
    c=[c;targets(i,3)*ones(threshold_interpolation,1)];
        else
     a=[a;linspace(targets(i,2),targets(i+1,2),abs(targets(i,1)-targets(i+1,1)))'];
     b=[b;linspace(targets(i,1),targets(i+1,1),abs(targets(i,1)-targets(i+1,1)))'];       
     c=[c;targets(i,3)*ones(abs(targets(i,1)-targets(i+1,1)),1)];
        end
    end
    
end

%% Finalize the Path

path_finalized=[b,a];
% path_finalized_3d=[path_finalized(:,1),path_finalized(:,2),3.5*ones(size(path_finalized,1),1)];
path_finalized_3d_2=[path_finalized(:,1),path_finalized(:,2),c];

path_finalized_3d_3=[path_finalized(:,1)+75,path_finalized(:,2)+70,c];

%% Plot the path 

% plot the 2D path 

figure(1),plot2(path_finalized);
figure(2),plot3(path_finalized_3d_2(:,1),path_finalized_3d_2(:,2),path_finalized_3d_2(:,3),'ro'),
hold on,
plot2(path_finalized,'b-');

%% Save Path to text file
 fileID = fopen('path.txt','w');
 
 for iii=1:size(path_finalized_3d_2,1)
     
 fprintf(fileID,'%f ,%f , %f , \n',path_finalized_3d_2(iii,1),path_finalized_3d_2(iii,2),path_finalized_3d_2(iii,3));
 
 end
%  fprintf(fileID,'%f , %f , %f , \n',path_finalized_3d_2(:,1),path_finalized_3d_2(:,2),path_finalized_3d_2(:,3))


