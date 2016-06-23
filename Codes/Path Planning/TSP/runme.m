% clean and clear

close all , clear all , clc;
%% Path 
path_unsorted=[86, 40, 1.75, 1
    ; 275, 137, 1.75, 1
    ; 128, 274, 1.75, 1
    ; 104, 2, 1.75, 0
    ; 178, 128, 1.75, 0
    ; 253, 280, 1.75, 1
    ; 317, 200, 1.75, 1
    ; 91, 283, 1.75, 0
    ; 125, 28, 1.75, 1
    ; 102, 194, 1.75, 0
    ; 349, 208, 1.75, 0
    ; 117, 61, 1.75, 0
    ; 150, 50, 1.75, 1
    ; 109, 131, 1.75, 0
    ; 135, 191, 1.75, 0
    ; 160, 38, 1.5, 0
    ; 2, 223, 1.75, 1
    ; 155, 215, 1.75, 1
    ; 132, 205, 1.75, 1
    ; 119, 157, 1.75, 1
    ; 16, 151, 1.75, 0
    ; 76, 5, 1.75, 1
    ; 316, 178, 1.75, 0
    ; 305, 282, 1.75, 0
    ; 313, 267, 1.75, 0
    ; 3, 197, 1.75, 0
    ; 91, 64, 1.75, 1
    ; 37, 152, 1.75, 1
    ; 272, 173, 1.75, 0
    ; 256, 128, 1.5, 1
    ; 79, 278, 1.75, 1
    ; 165, 276, 1.75, 1
    ; 104, 28, 1.75, 1
    ; 227, 283, 1.75, 1
    ; 213, 134, 1.75, 1
    ; 166, 243, 1.75, 1
    ; 191, 246, 1.75, 1
    ; 291, 203, 1.75, 1
    ; 282, 286, 1.5, 0
    ; 61, 233, 1.75, 0
    ; 353, 234, 1.75, 0
    ; 349, 280, 1.75, 0
    ; 174, 48, 1.75, 1
    ; 193, 63, 1.75, 0
    ; 79, 218, 1.75, 0
    ; 212, 95, 1.75, 0
    ; 243, 158, 1.75, 0
    ; 150, 271, 1.5, 1
    ; 59, 73, 1.75, 0
    ; 171, 192, 1.75, 1
    ; 52, 260, 1.75, 0
    ; 102, 105, 1.75, 0
    ; 91, 248, 1.75, 1
    ; 156, 135, 1.75, 1
    ; 312, 241, 1.75, 0
    ; 164, 164, 1.75, 0
    ; 62, 152, 1.75, 1
    ; 56, 47, 1.75, 0
    ; 286, 158, 1.75, 0
    ; 171, 221, 1.75, 0
    ; 100, 137, 1.75, 1
    ; 192, 280, 1.75, 0
    ; 135, 122, 1.75, 1
    ; 1, 157, 1.75, 1
    ; 350, 251, 1.75, 0
    ; 25, 182, 1.75, 0
    ; 327, 210, 1.75, 1
    ; 242, 102, 1.75, 0
    ; 25, 245, 1.75, 0
    ; 164, 94, 1.75, 1
    ; 80, 170, 1.75, 1
    ; 32, 215, 1.75, 0
    ; 134, 238, 1.75, 1
    ; 136, 10, 1.75, 1
    ; 149, 158, 1.75, 1
    ];

%% change the point to center
area_size=[20,15]; % camera ratio given by David.

factor_height=path_unsorted(:,3); % depends on the height to be multiplied by the previous area

area_covered_list=[];
for tot=1:size(path_unsorted,1)
    area_covered=area_size.*factor_height(tot);
    area_covered_list=[area_covered_list;area_covered];
    
    if(path_unsorted(tot,4)==0)
        path_unsorted_centered(tot,:)=[(path_unsorted(tot,1)+(area_covered(1)/2)) , (path_unsorted(tot,2)+(area_covered(2)/2))];
        
    else
        path_unsorted_centered(tot,:)=[(path_unsorted(tot,1)+(area_covered(2)/2)) , (path_unsorted(tot,2)+(area_covered(1)/2))];
    end
end
path_unsorted_centered(:,3)=path_unsorted(:,3);

%%
% take the 3D position in consideration

positions=[path_unsorted_centered(:,1),path_unsorted_centered(:,2),path_unsorted_centered(:,3)];

% run the main TSP sover method. 

[atour,path_sorted,atourlength]=tsp(positions);

%% plot the result of the path 

figure(10),subplot(122),title('path after planned'),plot3(path_sorted(:,1),path_sorted(:,2),path_sorted(:,3)),
hold on ,plot3(path_sorted(:,1),path_sorted(:,2),path_sorted(:,3),'ro');
subplot(121),title('original path without planning'),plot3(positions(:,1),positions(:,2),positions(:,3));

