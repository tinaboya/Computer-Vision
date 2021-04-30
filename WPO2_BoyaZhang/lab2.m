clear all; close all; %#ok<*CLALL>
%% Part 1
% Calibration
% Select select the points

% image_left = imread("left.jpg");
% imshow(image_left);
% [xl,yl] = ginput(12);
% 
% image_right = imread("right.jpg");
% imshow(image_right);
% [xr,yr] = ginput(12);

load('points.mat');
calpoints = load('calibration_points3.txt');
calpoints = calpoints';

% Matrix M (left)
Al = zeros(24,12);
for i=1:24
    if mod(i,2)
     Al(i,:) = [calpoints(1,(i+1)/2),calpoints(2,(i+1)/2),calpoints(3,(i+1)/2),1,0,0,0,0,-xl((i+1)/2)*calpoints(1,(i+1)/2),-xl((i+1)/2)*calpoints(2,(i+1)/2),-xl((i+1)/2)*calpoints(3,(i+1)/2),-xl((i+1)/2)];
    else
     Al(i,:) = [0,0,0,0,calpoints(1,i/2),calpoints(2,i/2),calpoints(3,i/2),1,-yl(i/2)*calpoints(1,i/2),-yl(i/2)*calpoints(2,i/2),-yl(i/2)*calpoints(3,i/2),-yl(i/2)];
    end
end
[Ul,Sl,Vl] = svd(Al);
Ml = Vl(:,12);
Ml = reshape(Ml,[4 3])';
Ml = Ml/Ml(end); %normalization

% Matrix M (right)
Ar = zeros(24,12);
for i=1:24
    if mod(i,2)
     Ar(i,:) = [calpoints(1,(i+1)/2),calpoints(2,(i+1)/2),calpoints(3,(i+1)/2),1,0,0,0,0,-xl((i+1)/2)*calpoints(1,(i+1)/2),-xr((i+1)/2)*calpoints(2,(i+1)/2),-xr((i+1)/2)*calpoints(3,(i+1)/2),-xr((i+1)/2)];
    else
     Ar(i,:) = [0,0,0,0,calpoints(1,i/2),calpoints(2,i/2),calpoints(3,i/2),1,-yl(i/2)*calpoints(1,i/2),-yr(i/2)*calpoints(2,i/2),-yr(i/2)*calpoints(3,i/2),-yr(i/2)];
    end
end
[Ur,Sr,Vr] = svd(Ar);
Mr = Vr(:,12);
Mr = reshape(Mr,[4 3])';
Mr = Mr/Mr(end); %normalization

% Coordinate system (left)
cs_l = Ml*[0 200 0 0; 0 0 200 0; 0 0 0 200;1 1 1 1];
for i=1:4
    cs_l(:,i) = cs_l(:,i)/cs_l(3,i); %normalization
end

% Points in the coordinate system (left)
points_l = Ml*[calpoints(1,:);calpoints(2,:);calpoints(3,:);ones(1,12)];
for i=1:12
    points_l(:,i) = points_l(:,i)/points_l(3,i); %normalization
end

% Coordinate system (right)
cs_r = Mr*[0 200 0 0; 0 0 200 0; 0 0 0 200;1 1 1 1];
for i=1:4
    cs_r(:,i) = cs_r(:,i)/cs_r(3,i); %normalization
end

% Points in the coordinate system (right)
points_r = Mr*[calpoints(1,:);calpoints(2,:);calpoints(3,:);ones(1,12)];
for i=1:12
    points_r(:,i) = points_r(:,i)/points_r(3,i); %normalization
end

% plot1
figure;
% left
subplot(1,2,1);
image_left = imread("left.jpg");
image_left = insertMarker(image_left,points_l(1:2,:)','x');
imshow(image_left); 
hold on;
for n = 1:12
text(points_l(1,n),points_l(2,n),num2str(n));
end
hold on;
line([cs_l(1,1),cs_l(1,2)],[cs_l(2,1),cs_l(2,2)]);
line([cs_l(1,1),cs_l(1,3)],[cs_l(2,1),cs_l(2,3)]);
line([cs_l(1,1),cs_l(1,4)],[cs_l(2,1),cs_l(2,4)]);
title('Left image');
axis on;
% right
subplot(1,2,2);
image_right = imread("right.jpg");
image_left = insertMarker(image_right,points_r(1:2,:)','x');
imshow(image_left); 
hold on;
for n = 1:12
text(points_r(1,n),points_r(2,n),num2str(n));
end
hold on;
line([cs_r(1,1),cs_r(1,2)],[cs_r(2,1),cs_r(2,2)]);
line([cs_r(1,1),cs_r(1,3)],[cs_r(2,1),cs_r(2,3)]);
line([cs_r(1,1),cs_r(1,4)],[cs_r(2,1),cs_r(2,4)]);
title('Right image');
axis on;
%% Part 2
% 3D reconstruction
% image_left = imread("left.jpg");
% imshow(image_left);
% [objects_xl,objects_yl] = ginput(18);
% image_right = imread("right.jpg");
% imshow(image_right);
% [objects_xr,objects_yr] = ginput(18);

load('objects.mat');
points1 = zeros(4,12);
points2 = zeros(4,18);

for i=1:12
    B = [xl(i)*Ml(3,:)-Ml(1,:);yl(i)*Ml(3,:)-Ml(2,:);xr(i)*Mr(3,:)-Mr(1,:);yr(i)*Mr(3,:)-Mr(2,:)];
    [U1,S1,V1] = svd(B);
    points1(:,i) =  V1(:,end)';
    points1(:,i) = points1(:,i)/points1(4,i); %normalization
end

for i=1:18
    C = [objects_xl(i)*Ml(3,:)-Ml(1,:);objects_yl(i)*Ml(3,:)-Ml(2,:);objects_xr(i)*Mr(3,:)-Mr(1,:);objects_yr(i)*Mr(3,:)-Mr(2,:)];
    [U1,S1,V1] = svd(C);
    points2(:,i) =  V1(:,end)';
    points2(:,i) = points2(:,i)/points2(4,i); %normalization
end
points = [points1 points2];

%plot2
figure;
% points in 3D world
plot3(points(1,:),points(2,:),points(3,:),'b*');
for n=1:12
hold on;
text(points1(1,n),points1(2,n),points1(3,n),[' ',num2str(n)])
end

% 3D figure
% The book
max = max(points1,[],2);
patch_x = [0 0 0 0];
patch_y = [max(2) 0 0 max(2)];
patch_z = [max(3) max(3) 0 0];
hold on;
patch(patch_x,patch_y,patch_z,[0.2 0.2 0.2],'FaceAlpha',0.5);
clear('patch_x','patch_y','patch_z');
patch_x = [0 0 max(1) max(1)];
patch_y = [0 0 0 0];
patch_z = [max(3) 0 0 max(3)];
hold on;
patch(patch_x,patch_y,patch_z,[0.2 0.2 0.2],'FaceAlpha',0.5);
clear('patch_x','patch_y','patch_z');

% The objects
% Cube1
patch_x = [points2(1,1) points2(1,4) points2(1,3) points2(1,2)];
patch_y = [points2(2,1) points2(2,4) points2(2,3) points2(2,2)];
patch_z = [points2(3,1) points2(3,4) points2(3,3) points2(3,2)];
hold on;
patch(patch_x,patch_y,patch_z,[0.8 0.8 0.8]);
clear('x_patch','y_patch','z_patch');
patch_x = [points2(1,1) points2(1,5) points2(1,6) points2(1,4)];
patch_y = [points2(2,1) points2(2,5) points2(2,6) points2(2,4)];
patch_z = [points2(3,1) points2(3,5) points2(3,6) points2(3,4)];
hold on;
patch(patch_x,patch_y,patch_z,[0.8 0.8 0.8]);
clear('x_patch','y_patch','z_patch');

patch_x = [points2(1,6) points2(1,7) points2(1,3) points2(1,4)];
patch_y = [points2(2,6) points2(2,7) points2(2,3) points2(2,4)];
patch_z = [points2(3,6) points2(3,7) points2(3,3) points2(3,4)];
hold on;
patch(patch_x,patch_y,patch_z,[0.8 0.8 0.8]);
clear('x_patch','y_patch','z_patch');

% Pyramid
patch_x = [points2(1,9) points2(1,8) points2(1,11)];
patch_y = [points2(2,9) points2(2,8) points2(2,11)];
patch_z = [points2(3,9) points2(3,8) points2(3,11)];
hold on;
patch(patch_x,patch_y,patch_z,[0.9 0 0]);
clear('x_patch','y_patch','z_patch');
patch_x = [points2(1,9) points2(1,11) points2(1,10)];
patch_y = [points2(2,9) points2(2,11) points2(2,10)];
patch_z = [points2(3,9) points2(3,11) points2(3,10)];
hold on;
patch(patch_x,patch_y,patch_z,[0.9 0 0]);
clear('x_patch','y_patch','z_patch');

% Cube2
patch_x = [points2(1,12) points2(1,15) points2(1,14) points2(1,13)];
patch_y = [points2(2,12) points2(2,15) points2(2,14) points2(2,13)];
patch_z = [points2(3,12) points2(3,15) points2(3,14) points2(3,13)];
hold on;
patch(patch_x,patch_y,patch_z,[0.7 0.7 0.5]);
clear('x_patch','y_patch','z_patch');
patch_x = [points2(1,12) points2(1,16) points2(1,17) points2(1,15)];
patch_y = [points2(2,12) points2(2,16) points2(2,17) points2(2,15)];
patch_z = [points2(3,12) points2(3,16) points2(3,17) points2(3,15)];
hold on;
patch(patch_x,patch_y,patch_z,[0.7 0.7 0.5]);
clear('x_patch','y_patch','z_patch');
patch_x = [points2(1,17) points2(1,18) points2(1,14) points2(1,15)];
patch_y = [points2(2,17) points2(2,18) points2(2,14) points2(2,15)];
patch_z = [points2(3,17) points2(3,18) points2(3,14) points2(3,15)];
hold on;
patch(patch_x,patch_y,patch_z,[0.7 0.7 0.5]);
clear('x_patch','y_patch','z_patch');

title('3D reconstruction');
xlabel('x');ylabel('y');zlabel('z');
axis tight equal;
grid on;  