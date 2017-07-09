function [rMatrices tVectors] = visual_odometry_mono(folderPath)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    rMatrices = [];
    tVectors = [];
    
    numFeatures = 500;
    
    % get list of images
    images = dir(strcat(folderPath, '/*.png'));
    
    % get the camera intrinsic and extrinsic calibration
    % from KITTI matlab devkit
    calib = loadCalibrationCamToCam(fullfile(folderPath,'calib_cam_to_cam.txt'));
    K = transpose(calib.K{1});
    K(2, 2) = K(2, 2) + 1.0;
    cameraParams = cameraParameters('IntrinsicMatrix', K);

    
    % take next two images and work on them
    while length(images) >= 2
        img_1_path = fullfile(images(1).folder, images(1).name);
        img_2_path = fullfile(images(2).folder, images(2).name);
        disp(img_1_path)
        %disp(img_2_path)
        % remove image that we are currently working on
        images(1) = [];
        
        img_1 = imread(img_1_path);
        img_2 = imread(img_2_path);
        
        if size(img_1, 3) == 3
            img_1 = rgb2gray(img_1);
            img_2 = rgb2gray(img_2);
        end

        % FEATURE EXTRACTION
        img_1_features = visual_odometry_mono_detectCorners(img_1, numFeatures);

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

%         tmp = cornerPoints(img_1_features);
%         figure;
%         imshow(img_1); hold on;
%         plot(tmp);
%         hold off;

        % ESSENTIAL MATRIX
        % uses MSAC
        img_1_features = cornerPoints(img_1_features);
        img_2_features = cornerPoints(img_2_features);
        [E, inliers] = estimateEssentialMatrix(img_1_features, img_2_features, cameraParams);

        inlierPoints1 = img_1_features(inliers);
        inlierPoints2 = img_2_features(inliers);
%         figure
%         showMatchedFeatures(img_1,img_2,inlierPoints1,inlierPoints2);
%         title('Inlier Matches')

        % - COMPUTE R and t
        % relativeCameraPose decomposes E and finds correct solution out of
        % the four for R (2x) and t (2x)
        [relativeOrientation,relativeLocation] = relativeCameraPose(E,cameraParams,inlierPoints1,inlierPoints2);
        
        %[rotationMatrix,translationVector] = cameraPoseToExtrinsics(relativeOrientation,relativeLocation)
        
        % assumption: we always travel forward
        % invert vector and matrix if vector is pointing in negative
        % direction
        if relativeLocation(1) < 0
            relativeLocation = relativeLocation * -1;
            relativeOrientation = inv(relativeOrientation);
        end
        rMatrices = cat(3, rMatrices, relativeOrientation);
        tVectors = [tVectors; relativeLocation];
    end
end

