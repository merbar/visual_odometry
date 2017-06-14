img_1 = rgb2gray(imread('data/img_1/0000000005.png'));
img_2 = rgb2gray(imread('data/img_1/0000000006.png'));

corners_img_1 = detectFASTFeatures(img_1, 'MinQuality', 0.30, 'MinContrast', 0.2);
corners_img_2 = detectFASTFeatures(img_2, 'MinQuality', 0.30, 'MinContrast', 0.2);

% PointsGPU = detectFASTFeatures(gpuI,___) perform operation on a graphics processing unit (GPU), where gpuI is a gpuArray object that contains a 2-D grayscale input image. The output is a cornerPoints object. This syntax requires the Parallel Computing Toolbox™.

imshow(img_1); hold on;
plot(corners.selectStrongest(50));

% - FEATURE EXTRACTION
%   make sure features are evenly distributed across image
%   use bucketing to extract features => sample 200x200px regions
% - FEATURE MATCHING
%   tracker = vision.PointTracker('MaxBidirectionalError', 1);
%   initialize(tracker, points1_l.Location, I1_l);
%   [points2_l, validity] = step(tracker, I2_l);
%   track these points as long as the number of points do not drop below a
%   particular threshold?!
%   Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
% - INLIER DETECTION
%   only keep high quality features
% - COMPUTE R and t