function [u,v] = LucasKanadeOpticalFlow(img1, img2, WindowSize, MaxIter)
% LucasKanadeMethod

% Partial derivatives
Ix_p = conv2(img1,[-1 1; -1 1], 'valid'); % partial on x
Iy_p = conv2(img1, [-1 -1; 1 1], 'valid'); % partial on y
It_p = conv2(img1, ones(2), 'valid') + conv2(img2, -ones(2), 'valid'); % partial on t

% Perform iteration

% Initialize u v
u = zeros(size(img1));
v = zeros(size(img2));

% Find A and b, then get u and v
w = round(WindowSize/2);
for i = w+1:size(Ix_p,1)-w
   for j = w+1:size(Ix_p,2)-w
      Ix = Ix_p(i-w:i+w, j-w:j+w);
      Iy = Iy_p(i-w:i+w, j-w:j+w);
      It = It_p(i-w:i+w, j-w:j+w);
      
      Ix = Ix(:); % row to column
      Iy = Iy(:); 
      b = -It(:); % b 
      A = [Ix Iy]; % A
      nu = pinv(A)*b; % find the velocity solution

      u(i,j)=nu(1); % u
      v(i,j)=nu(2); % v
   end
end
end

