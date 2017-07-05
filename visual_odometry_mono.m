function [] = visual_odometry_mono(folderPath)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    
    % get list of images
    images = dir(strcat(folderPath, '/*.png'));
    img_1_path = strcat(images(1).folder, '/', images(1).name);
    img_2_path = strcat(images(2).folder, '/', images(2).name);
    
    img_1 = rgb2gray(imread(img_1_path));
    img_2 = rgb2gray(imread(img_2_path));
    
    % FEATURE EXTRACTION
    % extract features from each
    img_1_features = visual_odometry_mono_detectCorners(img_1);
    %img_2_features = visual_odometry_mono_detectCorners(img_2);
    
    % FEATURE MATCHING
    % Just tracking the points from one frame to the next, and then again
    % doing the detection part, but in a better implmentation, one would
    % track these points as long as the number of points do not drop below
    % a particular threshold.
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(tracker, img_1_features.Location, img_1);
    [img_2_features, validity] = step(tracker, img_2);
    bad_match_indeces = validity(:)==0;
    img_1_features = img_1_features.Location;
    img_1_features(bad_match_indeces,:) = [];
    img_2_features(bad_match_indeces,:) = [];
    
    figure;
    imshow(img_1); hold on;
    plot(img_1_features);
    hold off;
    
%     size(img_1_features.Location)
%     size(img_2_features)
    
%     img_2_features = img_2_features';
%     figure;
%     imshow(img_2); hold on;
%     plot(img_2_features(1,:), img_2_features(2,:), 'gx');
%     hold off;

    
    
   
    %   Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
    % - INLIER DETECTION
    %   only keep high quality features
    % - COMPUTE R and t
    
end

