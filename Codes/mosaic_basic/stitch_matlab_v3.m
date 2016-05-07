%% structure of the image with the poses should be created ! :|

%% Intialize

clear all , close all , clc;

%path = '~/Downloads/Download/Image Mosaicing';

path_code='/home/strub';

path_images='/home/strub/Image_objects_ICPR_rotationEdited2/';

path=path_images;

%im=fullfile(path,'lena_color.jpg');

srcFiles = dir(strcat(path,'*.jpg'));  % the folder in which ur images exists

for i = 1 : length(srcFiles)
    %I=zeros(length(srcFiles),512,512);                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    %I(1:length(srcFiles))=zeros(512,512);
    
    filename = strcat(path,srcFiles(i).name);
    
    I = imread(filename);
    
    %Images{i}=I;
    %Images_Gray=rgb2gray(Images{i});
    %figure, imshow(Images{i});
    
    img_cell{i}=I;

end
%%

modif=zeros(22,1);

for i=1:size(srcFiles,1)
    %[lon]=sortrows(srcFiles(i).name)
    namy=srcFiles(i).name;
    mo=strrep(namy, '.jpg','');
    mo=strrep(mo, 'frame0000_','')
    modif(i)=str2double(mo)
    
end

%%
[sop,idx]=sort(modif,1);

%%
imageee=img_cell;
for i=1:22
    imageee{i}=img_cell{idx(i)};
end


%%

% [xy]= [150,250;450,250;750,250];
% xy=[xy(:,2) xy(:,1)];

%%%%%%%%%% OR 
%[xy]=[250,150;250,450;250,750];


 xyzO=[62, 65, 1.5, 1  ;
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

%%
% for i=1:size(xyzO,1);
%     if(xyzO(i,4)==1)
%         imageee{i}=imrotate(imageee{i},90);
%     end
% end

%%
xyz=[xyzO(:,1),xyzO(:,2),xyzO(:,3)];
%%

[image_size]=size(imageee{3});
image_area=[20 15];
factor_of_height=1.5;
image_area=image_area*factor_of_height;

pose_multiple=[image_size(1),image_size(2)]./image_area; % [image_area(2),image_area(1)]

% xyz=[xyz(:,1)+(image_size(1,1)/2), (xyz(:,2)+(image_size(1,2)/2)),xyz(:,3)];

% xyz=[xyz(:,1)+(image_size(1,1)), (xyz(:,2)+(image_size(1,2))),xyz(:,3)];

[xy]=[xyz(:,1:2)];
room_size=[150 140];

%[xy]=[room_size(1)-xy(:,1),room_size(2)-xy(:,2)];

xy(:,1)=xy(:,1).*pose_multiple(1);
xy(:,2)=xy(:,2).*pose_multiple(2);

%[xy]=[xy].*pose_multiple(1);

%%%%%%%%%%%%%%%%%%%%  TRIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%img1=double(img1);

%img_array=cell(img2,img1,img2,img1);

%img_array=[img2,img1,img2,img1];
%%%%%%%%%%%%%%%%%%% END TRIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%img_cell=cell(Images_Gray);

%%

% [h1, w1, x1] = size(img1);
% [h2, w2, x2] = size(img2);
% [h3, w3, x3] = size(img3);

for jj=1:length(imageee)

[h(jj), w(jj), x(jj)]=size(imageee{jj});
end
%[h4, w4, x4] = size(img4);

%%

vis=zeros(h(1)*12,w(1)*12,x(1));
vis_1=vis(:,:,1);
vis_3=vis_1;
%%
%if (length(xy)==length(img_array))
for ii=0:length(xy)-1

   % vis(xy(i,1):h1+xy(i,1)-1, xy(i,2):w1+xy(i,2)-1,:) = img1(:,:,:);

   % vis(x_now:h1+x_now-1, y_now:w1+y_now-1,:) = img_array(:,ii*h1+1:((ii*h1)+h1),:);
     
   %vis(x_now:h1+xy(ii+1,1)-1, y_now:w1+y_now-1,:) = img_array(:,ii*h1+1:((ii*h1)+h1),:);
   
   x_now=xy(ii+1,1);
   y_now=xy(ii+1,2);
   if (x_now==0)
       x_now=0.1;
   end
   if (y_now==0)
       y_now=0.1;
   end
   vis_1(x_now:h(ii+1)+x_now-1,y_now:w(ii+1)+y_now-1,1)=rgb2gray(imageee{ii+1});
   
   vis2=(vis(:,:,1));
   figure(44),subplot(121),imshow(vis_1,[]),subplot(122),
   imshow(vis_1(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,1),[]);
   
   image_output=vis_3(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,1);
   
   image_output=image_output+double(rgb2gray(imageee{ii+1}));
   
%    figure(44),subplot(121),imshow(vis_1,[]),subplot(122),
%    imshow(vis_1(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,1),[]);
%    
   
   vis_5R(x_now:h(ii+1)+x_now-1,y_now:w(ii+1)+y_now-1)=imageee{ii+1}(:,:,1);
   vis_5G(x_now:h(ii+1)+x_now-1,y_now:w(ii+1)+y_now-1)=imageee{ii+1}(:,:,2);
   vis_5B(x_now:h(ii+1)+x_now-1,y_now:w(ii+1)+y_now-1)=imageee{ii+1}(:,:,3);
   

   %waitforbuttonpress;
   
   %figure(8),imshow(vis_3,[]);
    %vis(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,1) = img_cell{ii+1}(:,:,1);
    
    %vis(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,2) = img_cell{ii+1}(:,:,2);
    %vis(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,3) = img_cell{ii+1}(:,:,3);
    
    
    %figure(2),subplot(2,(length(xy)-1/3),ii+1),imshow(vis(x_now:h(ii+1)+x_now-1, y_now:w(ii+1)+y_now-1,:),[]);
end

% else 
%     print('the number of points and the images are not the same');
% end

%vis(xy(1,1):h1+xy(1,1)-1, xy(1,2):w1+xy(1,2)-1,:) = img1(:,:,:);


%%
   vis_5(:,:,1)=vis_5R;
   vis_5(:,:,2)=vis_5G;
   vis_5(:,:,3)=vis_5B;
 
%%


%vis(1:h2, w1:w1+w2-1,:) = img2;

figure(2),imshow(vis_5,[])


%% Read text file of co-ordinates

% it will only work if the file is text and is comma separated.

% if you want to work with space then change the formatSpec remove the ,
% and add space .

% sizeA=[2 Inf];
% %sizeA=[Inf 2];
% 
% fileId=fopen('coordinates_dummy.txt','r') ;
% formatSpec = '%d,%d';
% 
% [A size]=fscanf(fileId,formatSpec,sizeA);
% A=A'
% fclose(fileId);