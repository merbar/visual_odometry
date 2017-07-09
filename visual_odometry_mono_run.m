% https://avisingh599.github.io/vision/visual-odometry-full/
% https://github.com/avisingh599/vo-howard08/blob/master/src/visodo.m

clear
[rMatrices tVectors] = visual_odometry_mono('data/img_1');

% assert that we have the same number of rotation matrices and translation
% vectors
assert(size(tVectors, 1) == size(rMatrices, 3));

% translations have unknown scale factor (they are unit vectors)
% we'd have to get the current forward velocity from the ground truth, but
% I'll just set it to 3m/s  for now
speed = 3.0;

% create array of 2D points starting from origin
cur_coord = [0 0 1];
vehicle_coordinates = cur_coord(1:2);

% progress through each data point
for i = 1:1:size(tVectors, 1)-1
    rotMat = rMatrices(:,:,i);
    transVec = tVectors(i,:);
    rotTransMat = rotMat;
    rotTransMat(1,3) = transVec(1);
    rotTransMat(2,3) = transVec(2);
    rotTransMat(3,3) = 1.0;
    rotTransMat(3,1) = 0;
    rotTransMat(3,2) = 0;
    % vector and matrix aren't aligned the same
    rotTransMat = rotTransMat';
    cur_coord = cur_coord * rotTransMat;
    %cur_coord(3) = 1.0;
    coord2d = cur_coord(1:2);
    vehicle_coordinates = [vehicle_coordinates; coord2d];
end