clear variables;
clc;
close all;

load('Data.mat');
% for i=1:size(I,3)
%     figure(100+i);
%     imshow(flipud(I(:,:,i)));
%     title(sprintf('Image %02d',i));
% end
[p,q] = PhotometricStereo(I,shadow_mask,L);

%% Visualize the normal map
n(:,:,1) = p;
n(:,:,2) = q;
n(:,:,3) = -1*ones(size(p));
figure; imshow(n); axis xy; drawnow;
title('Normal map');

%% Estimate depth map from the normal vectors.
fprintf('Estimating depth map from normal vectors...\n');
Z = DepthFromGradient(p, q);
% Visualize depth map.
figure;
surf(Z, 'EdgeColor', 'None', 'FaceColor', [0.5 0.5 0.5]);
axis equal; camlight;
view(-75, 30);
title('Shape reconstruction');