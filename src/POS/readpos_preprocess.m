%% pre-process pos data

dataset = 'E:/xiao.teng/sfmIPI/dataset/16images';
POS =load([dataset, '/POS/pos.txt']);
fileCR = fopen([dataset, '/POS/posCR.txt'], 'w');
formatSpec ='%f %f %f %f %f %f %f %f %f %f %f %f\n';
fileCRcg = fopen([dataset, '/POS/posCR_n.txt'], 'w');
%% draw original pos(pos_tR) to posCR
for i=1:size(POS,1)
    posi = POS(i,:);
    t= posi(1:3)';
    R = vec2mat(posi(4:12),3);
    C=-R' * t;    
    drawCameraRC([R,C], 1, 1, 1, 10, 1); axis equal;
    %%output CR style
    fprintf(fileCR,formatSpec,[C',posi(4:12)]);
    POS(i,1:3) = C';
end
fclose(fileCR);
%%%%center of gravity
POScg = POS;
cg = mean(POScg(:,1:3));
POScg(:,1:3)=POScg(:,1:3)-cg;

for j=1:size(POScg,1)
 posj = POScg(j,:);
 fprintf(fileCRcg,formatSpec,posj);    
end
fclose(fileCRcg);
%% do ba in openmvg

%% recover the geodesic coordinate camera 
posCR_n = load([dataset,'/sfme.json/sfm_pos/outBA/cameras_poseCR_n.txt']);
posCR=posCR_n;
posCR(:,1:3) = posCR(:,1:3)+cg;
file_tR = fopen([dataset,'/sfme.json/sfm_pos/outBA/cameras_posetR.txt'],'w');
for k=1:size(posCR,1)
    C=posCR(k,1:3)';
    R = vec2mat(posCR(k,4:12),3);
    t = -R*C;
    fprintf(file_tR,formatSpec,[t',posCR(k,4:12)] );    
end
fclose(file_tR);

%%
points_n = load([dataset, '/sfme.json/sfm_pos/outBA/structure_3Dpoints_n.txt']);
points = points_n;
points(:,1:3) = points(:,1:3)+cg;
file_points = fopen([dataset, '/sfme.json/sfm_pos/outBA/structure_3Dpoints.txt'],'w');
for m=1:size(points,1)
    fprintf(file_points, '%f %f %f\n', points(m,1:3));    
end
fclose(file_points);


