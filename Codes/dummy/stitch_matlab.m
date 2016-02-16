%% Intialize

clear all , close all , clc;

path = '~/Downloads/Download/Image Mosaicing';
im=fullfile(path,'lena_color.jpg');
img1 = imread(im);

[xy]= [50,50;1200,1100;2300,200;1500,1150];

%img1=double(img1);

img2=img1;

%img_array=cell(img2,img1,img2,img1);
img_array=[img2,img1,img2,img1];

%%

[h1, w1, x1] = size(img1);
[h2, w2, x2] = size(img2);

%%

vis=zeros(h1*4,w1*4,x1);

%%
%if (length(xy)==length(img_array))
for ii=0:length(xy)-1
%     vis(xy(i,1):h1+xy(i,1)-1, xy(i,2):w1+xy(i,2)-1,:) = img1(:,:,:);

    vis(xy(ii+1,1):h1+xy(ii+1,1)-1, xy(ii+1,2):w1+xy(ii+1,2)-1,:) = img_array(:,ii*h1+1:((ii*h1)+h1),:);


end

% else 
%     print('the number of points and the images are not the same');
% end

%vis(xy(1,1):h1+xy(1,1)-1, xy(1,2):w1+xy(1,2)-1,:) = img1(:,:,:);

%%

vis(1:h2, w1:w1+w2-1,:) = img2;

imshow(vis,[])


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
