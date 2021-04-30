function [p,q] = PhotometricStereo(I,mask,L)
% Get the size of I
I_1 = I.*mask;
[N1, N2, M] = size(I_1);
% Calculate the "L"
L_t = L';
L_1 = ((L_t'*L_t)^-1)*L_t';
% Define size of the output
p = zeros(N1,N2);
q = zeros(N1,N2);

for i=1:N1
    for j=1:N2
        I_2 = reshape(I_1(i,j,:),M,1); 
        n = L_1*I_2; % L times I
        if n(3)
            p(i,j) = n(1)/-n(3);
            q(i,j) = n(2)/-n(3); 
        else
            p(i,j) = n(1);
            q(i,j) = n(2);
        end
    end
end
end

