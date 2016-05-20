clear all;
load p_2d_;
p_2d=p_2d_;

[T, J] = size(p_2d); T = T/2;

% 2D motion resulting from orthographic projection (Eq (1))
% p2_obs = P3_gt(1:2*T, :);

% runs the non-rigid structure from motion algorithm
use_lds = 1;
max_em_iter = 60;
tol = 0.0001;
K = 3; % number of deformation shapes
% Zcoords_gt = P3_gt(2*T+1:3*T,:) - mean(P3_gt(2*T+1:3*T,:),2)*ones(1,J);
% Zdist = max(Zcoords_gt,[],2) - min(Zcoords_gt,[],2); % size of the 3D shape along the Z axis for each time frame
MD = zeros(T,J);

[P3, S_hat, V, RO, Tr, Z] = em_sfm(p_2d, MD, K, use_lds, tol, max_em_iter);


for i=1:139
    
    scatter3(P3(i,:),P3(i+139,:),P3(i+139*2,:),'ro');axis([-300 300 -300 300 -200 200]);
    pause(0.2);
end
