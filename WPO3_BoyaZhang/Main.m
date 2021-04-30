function Main()
    close all
    clearvars
    clc

    % Choose LK parameters
    WindowSize  =   11;
    MaxIter     =   3; %for the reuse of the solution

    img1 = double(imread('image1.png'))/255.0;
    img2 = double(imread('image2.png'))/255.0;

    %Compute optical flow using LK algorithm
    [u,v] = LucasKanadeOpticalFlow(img1, img2, WindowSize, MaxIter); %Doens't work :-)
    flow_img = draw_flow_rgb(u,v);
    figure();
    imshow(flow_img);
end

function img = draw_flow_rgb(u,v)
    flow(:,:,1) = u;
    flow(:,:,2) = v;
    dMax = max(abs(flow(:)));                  % max displacement
    img = bsxfun(@times, flow, cat(3,1,-1));   % U, -V
    img = (img - (-dMax)) / (dMax - (-dMax));  % map values
    img(:,:,3) = 0;
    img = uint8(255 * img);
end
