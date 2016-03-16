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

a=[];
b=[];
c=[];
for i=1:size(targets,1)-1
    
    if (abs(targets(i,2)-targets(i+1,2))>abs(targets(i,1)-targets(i+1,1)))
        
    a=[a;linspace(targets(i,2),targets(i+1,2),abs(targets(i,2)-targets(i+1,2)))'];
    b=[b;linspace(targets(i,1),targets(i+1,1),abs(targets(i,2)-targets(i+1,2)))'];
    c=[c;targets(i,3)*ones(abs(targets(i,2)-targets(i+1,2)),1)];
    
      scale=linspace(targets(i+1,2),targets(i,2),targets(i,2)-targets(i+1,2))';
      
    else
     a=[a;linspace(targets(i,2),targets(i+1,2),abs(targets(i,1)-targets(i+1,1)))'];
     b=[b;linspace(targets(i,1),targets(i+1,1),abs(targets(i,1)-targets(i+1,1)))'];       
     c=[c;targets(i,3)*ones(abs(targets(i,1)-targets(i+1,1)),1)];
    
    end
    
end

% path_finalized=[b,a];
% path_finalized_3d=[path_finalized(:,1),path_finalized(:,2),3.5*ones(size(path_finalized,1),1)];
path_finalized_3d_2=[path_finalized(:,1),path_finalized(:,2),c];

