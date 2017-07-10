% https://avisingh599.github.io/vision/visual-odometry-full/
% https://github.com/avisingh599/vo-howard08/blob/master/src/visodo.m

clear
[rMatrices tVectors] = visual_odometry_mono('data/debug');

% assert that we have the same number of rotation matrices and translation
% vectors
assert(size(tVectors, 1) == size(rMatrices, 3));

% translations have unknown scale factor (they are unit vectors)
% we'd have to get the current forward velocity from the ground truth, but
% I'll just set it to 3m/s  for now
speed = 3.0;

% create array of 2D points starting from origin
cur_coord = [0 0 0];
cur_orient = eye(3);
grid_coordinates = cur_coord;

% progress through each data point
for i = 1:1:size(tVectors, 1)
    R = rMatrices(:,:,i);
    t = tVectors(i,:);
%     rotTransMat = rotMat;
%     rotTransMat(1,3) = transVec(1);
%     rotTransMat(2,3) = transVec(2);
%     rotTransMat(3,3) = 1.0;
%     rotTransMat(3,1) = 0;
%     rotTransMat(3,2) = 0;
    % vector and matrix aren't aligned the same
    cur_coord = cur_coord + t;
    cur_orient = cur_orient * R;
    grid_coordinates = [grid_coordinates; cur_coord];
end