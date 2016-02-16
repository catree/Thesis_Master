%% Intialize

clear all , close all , clc;

path = '/home/mark/Downloads/Download/Image Mosaicing';
im=fullfile(path,'lena_color.jpg');
img1 = imread(im);
[xy]= [50,50;200,100;300,200;500,150];

%img1=double(img1);

img2=img1;

img_array=[img2,img1,img2,img1];

%%

[h1, w1, x1] = size(img1);
[h2, w2, x2] = size(img2);

%%

vis=zeros(h1*2,w1*2,x1);

%%

vis(xy(1,1):h1+xy(1,1)-1, xy(1,2):w1+xy(1,2)-1,:) = img1(:,:,:);

%%

vis(1:h2, w1:w1+w2-1,:) = img2;

imshow(vis,[])
