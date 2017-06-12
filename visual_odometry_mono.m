img_gray = rgb2gray(img);
corners = detectFASTFeatures(img_gray, 'MinQuality', 0.00, 'MinContrast', 0.1);