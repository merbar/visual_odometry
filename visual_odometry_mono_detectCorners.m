function [features points] = visual_odometry_mono_detectCorners(img, num_features)
    %visual_odometry_mono_detectCorners Detects evenly distributed corners 
    % across image
    %   Goes through image in chunks and extracts a max number of features
    %   per bucket, leading to a more even distribution.
    [img_size_y, img_size_x] = size(img);
    bucket_size = 100;
    x_bucket_count = ceil(img_size_x / bucket_size);
    y_bucket_count = ceil(img_size_y / bucket_size);
    
    features_per_bucket = ceil(num_features / (x_bucket_count * y_bucket_count));

    features = [];
    points = [];
    for y = 1:1:y_bucket_count
        bucket_y_start = (y-1) * bucket_size + 1;
        if y == y_bucket_count
            bucket_y_end = bucket_y_start + mod(img_size_y, bucket_size);
        else
            bucket_y_end = bucket_y_start + bucket_size;
        end
        for x = 1:1:x_bucket_count
            bucket_x_start = (x-1) * bucket_size + 1;
            if x == x_bucket_count
                bucket_x_end = bucket_x_start + mod(img_size_x, bucket_size);
            else
                bucket_x_end = bucket_x_start + bucket_size;
            end
            roi = [bucket_x_start bucket_y_start bucket_x_end-bucket_x_start bucket_y_end-bucket_y_start];
            crop_corners = detectSURFFeatures(img, 'MetricThreshold', 500, 'ROI', roi);
            crop_corners = detectHarrisFeatures(img, 'ROI', roi);
            %crop_corners = crop_corners.selectStrongest(features_per_bucket);
            tmp = crop_corners.Location;
            tmp = SURFPoints(tmp);
            [valid_features, valid_corners] = extractFeatures(img, tmp, 'Upright', true);
            features = [features; valid_features];
            points = [points; crop_corners.Location];
        end
    end
    points = SURFPoints(points);
end
