function [corners] = visual_odometry_mono_detectCorners(img)
    [img_size_y, img_size_x] = size(img);
    bucket_size = 200;
    x_bucket_count = ceil(img_size_x / bucket_size);
    y_bucket_count = ceil(img_size_y / bucket_size);

    corners = [];
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
            crop_corners = detectFASTFeatures(img, 'MinQuality', 0.30, 'MinContrast', 0.2, 'ROI', roi);
            corners = [corners; crop_corners.Location];
        end
    end
    corners = cornerPoints(corners);
end
