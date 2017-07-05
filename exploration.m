clear

img_1 = rgb2gray(imread('data/img_1/0000000005.png'));
img_2 = rgb2gray(imread('data/img_1/0000000006.png'));

roi = [500 1 300 300];
crop_corners = detectFASTFeatures(img_1, 'MinQuality', 0.30, 'MinContrast', 0.2, 'ROI', roi);
%crop_corners = detectFASTFeatures(img_1, 'MinQuality', 0.30, 'MinContrast', 0.2);
%corners = [corners; crop_corners.Location];

%corners = corners';
figure;
imshow(img_1); hold on;
plot(crop_corners);
hold off;



% corners_img_1_all = detectFASTFeatures(img_1, 'MinQuality', 0.30, 'MinContrast', 0.2);
% %corners_img_2 = detectFASTFeatures(img_2, 'MinQuality', 0.30, 'MinContrast', 0.2);
% 
% figure;
% imshow(img_1); hold on;
% %plot(corners_img_1_all.selectStrongest(50));
% plot(corners_img_1_all);
% hold off;
% 
% img_size = size(img_1);
% img_size_x = img_size(2);
% img_size_y = img_size(1);
% bucket_size = 200;
% x_bucket_count = ceil(img_size_x / bucket_size);
% y_bucket_count = ceil(img_size_y / bucket_size);
% 
% y_bucket_count_2 = floor(linspace(1, img_size_y, bucket_size));
% x_bucket_count_2 = floor(linspace(1, img_size_x, bucket_size));
% 
% img = img_1;
% 
% crop_debug_arr = [];
% corners = [];
% for y = 1:1:y_bucket_count
%     bucket_y_start = (y-1) * bucket_size + 1;
%     if y == y_bucket_count
%         bucket_y_end = bucket_y_start + mod(img_size_y, bucket_size);
%     else
%         bucket_y_end = bucket_y_start + bucket_size;
%     end
%     for x = 1:1:x_bucket_count
%         bucket_x_start = (x-1) * bucket_size + 1;
%         if x == x_bucket_count
%             bucket_x_end = bucket_x_start + mod(img_size_x, bucket_size);
%         else
%             bucket_x_end = bucket_x_start + bucket_size;
%         end
%         crop_debug_arr = [crop_debug_arr; [bucket_x_start ,bucket_x_end,bucket_y_start,bucket_y_end]];
%         roi = [bucket_x_start bucket_y_start bucket_x_end-bucket_x_start bucket_y_end-bucket_y_start];
%         crop_corners = detectFASTFeatures(img, 'MinQuality', 0.30, 'MinContrast', 0.2, 'ROI', roi);
%         corners = [corners; crop_corners.Location];
%     end
% end
% 
% corners = corners';
% figure;
% imshow(img_1); hold on;
% plot(corners(1,:), corners(2,:), 'gx');
% hold off;



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