%% Intialize

clear all , close all , clc;

%path = '~/Downloads/Download/Image Mosaicing';

path='/home/strub/scratch/Thesis_Master/Codes/dummy/';

%im=fullfile(path,'lena_color.jpg');

srcFiles = dir(strcat(path,'*.jpg'));  % the folder in which ur images exists

for i = 1 : length(srcFiles)
    %I=zeros(length(srcFiles),512,512);
    %I(1:length(srcFiles))=zeros(512,512);
    
    filename = strcat(path,srcFiles(i).name);
    I = imread(filename);
    Images{i}=I;
    Images_Gray=rgb2gray(I);
    figure, imshow(Images{i});
end


% read : http://fr.mathworks.com/help/images/ref/imresize.html

%img3 = imresize(img3, 0.5);

%%

% [xy]= [150,250;450,250;750,250];
% xy=[xy(:,2) xy(:,1)];

%%%%%%%%%% OR 
[xy]=[250,150;250,450;250,750];

%%%%%%%%%%%%%%%%%%%%  TRIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%img1=double(img1);

%img_array=cell(img2,img1,img2,img1);

%img_array=[img2,img1,img2,img1];
%%%%%%%%%%%%%%%%%%% END TRIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N=4;
img_cell=cell(1,N);
img_cell{1}=img1;
img_cell{2}=img2;
img_cell{3}=img3;

%img_cell{4}=img4;


%%

% [h1, w1, x1] = size(img1);
% [h2, w2, x2] = size(img2);
% [h3, w3, x3] = size(img3);

for jj=1:length(img_cell)-1

[h(jj), w(jj), x(jj)]=size(img_cell{jj});
end
%[h4, w4, x4] = size(img4);

%%

vis=zeros(h(1)*2.5,w(1)*4.5,x(1));

%%
%if (length(xy)==length(img_array))
for ii=0:length(xy)-1

   % vis(xy(i,1):h1+xy(i,1)-1, xy(i,2):w1+xy(i,2)-1,:) = img1(:,:,:);

   % vis(xy(ii+1,1):h1+xy(ii+1,1)-1, xy(ii+1,2):w1+xy(ii+1,2)-1,:) = img_array(:,ii*h1+1:((ii*h1)+h1),:);
     
   %vis(xy(ii+1,1):h1+xy(ii+1,1)-1, xy(ii+1,2):w1+xy(ii+1,2)-1,:) = img_array(:,ii*h1+1:((ii*h1)+h1),:);
    

    vis(xy(ii+1,1):h(ii+1)+xy(ii+1,1)-1, xy(ii+1,2):w(ii+1)+xy(ii+1,2)-1) = img_cell{ii+1};
    
end

% else 
%     print('the number of points and the images are not the same');
% end

%vis(xy(1,1):h1+xy(1,1)-1, xy(1,2):w1+xy(1,2)-1,:) = img1(:,:,:);

%%

%vis(1:h2, w1:w1+w2-1,:) = img2;

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


