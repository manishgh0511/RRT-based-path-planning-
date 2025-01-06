clc;
clear;
close all;

%% 1. Camera Parameters
f = 72.1; % Focal length in pixels
B = 0.424; % Baseline distance in meters

%% 2. Read and Preprocess the Images
% Replace these lines with the actual paths to your images
imgLeft = imread('Left_2_cropped01.png');
imgRight = imread('Right_2_cropped.png');

% Convert images to grayscale
imgLeftGray = rgb2gray(imgLeft);
imgRightGray = rgb2gray(imgRight);

% Apply Gaussian smoothing
sigma = 1.0; % Gaussian filter sigma
imgLeftSmooth = imgaussfilt(imgLeftGray, sigma);
imgRightSmooth = imgaussfilt(imgRightGray, sigma);

% Display smoothed images (optional)
figure;
subplot(1,2,1);
imshow(imgLeftSmooth);
title('Smoothed Left Image');
subplot(1,2,2);
imshow(imgRightSmooth);
title('Smoothed Right Image');

%% 3. Feature Matching
% Detect feature points in both images
pointsLeft = detectSURFFeatures(imgLeftSmooth);
pointsRight = detectSURFFeatures(imgRightSmooth);

% Extract feature descriptors
[featuresLeft, validPointsLeft] = extractFeatures(imgLeftSmooth, pointsLeft);
[featuresRight, validPointsRight] = extractFeatures(imgRightSmooth, pointsRight);

% Match features between images
indexPairs = matchFeatures(featuresLeft, featuresRight);
matchedPointsLeft = validPointsLeft(indexPairs(:,1), :);
matchedPointsRight = validPointsRight(indexPairs(:,2), :);

% Display matched features
figure;
showMatchedFeatures(imgLeftSmooth, imgRightSmooth, matchedPointsLeft, matchedPointsRight);
title('Matched Points');

%% 4. Compute Geometric Transformation
% Estimate the transformation matrix using matched points
[tform, inlierIdx] = estimateGeometricTransform(matchedPointsRight, matchedPointsLeft, 'projective');

% Transform the right image to align with the left image
outputImage = imwarp(imgRightSmooth, tform, 'OutputView', imref2d(size(imgLeftSmooth)));
alignedRightImage = imwarp(imgRightGray, tform, 'OutputView', imref2d(size(imgLeftGray)));

% Display the aligned images
figure;
imshowpair(imgLeftGray, alignedRightImage, 'blend');
title('Aligned Left and Right Images');

%% 5. Create a Single Panoramic Image
% Create a single image by combining the left and transformed right images
panoramicImage = max(imgLeftGray, alignedRightImage);

% Display the panoramic image
figure;
imshow(panoramicImage);
title('Panoramic Image');

%% 6. Edge Detection Using Laplacian Filter
% Define the standard deviation for Gaussian smoothing
sigma = 2;

% Smooth the panoramic image
img_smoothed = imgaussfilt(panoramicImage, sigma);

% Define the Laplacian filter
laplacian_filter = fspecial('laplacian', 0.2);

% Apply the Laplacian filter to the smoothed image to detect edges
edges = imfilter(img_smoothed, laplacian_filter, 'replicate');

% Apply a threshold to get binary edge image
threshold = graythresh(edges); % Otsu's method for automatic thresholding
binary_edges = imbinarize(edges, threshold);

% Display binary edge image to verify detection
figure;
imshow(binary_edges);
title('Edges Detected using Laplacian');

%% 7. Determine Dynamic Minimum Area Threshold Using Percentiles
% Detect connected components in the binary edge image
stats = regionprops(binary_edges, 'BoundingBox', 'Area');

% Compute areas of detected components
areas = [stats.Area];

% Determine the percentile threshold for dynamic minimum area
if ~isempty(areas)
    percentile_threshold = prctile(areas, 98); % Use 98th percentile to reduce detection of small obstacles
else
    percentile_threshold = 0; % Default to 0 if no areas are detected
end

% Filter out small regions based on the dynamic minimum area threshold
filtered_stats = stats([stats.Area] >= percentile_threshold);

% Display the results on the panoramic image
figure;
imshow(panoramicImage);
title('Detected Obstacles with Bounding Boxes');
hold on;

% Draw bounding boxes around detected obstacles
for k = 1:length(filtered_stats)
    bb = filtered_stats(k).BoundingBox;
    rectangle('Position', bb, 'EdgeColor', 'r', 'LineWidth', 2);
end

hold off;

% Additional diagnostic messages
if isempty(filtered_stats)
    disp('No obstacles detected. Try adjusting the parameters.');
else
    disp(['Detected ' num2str(length(filtered_stats)) ' obstacles.']);
end

%% 8. Create and Save Binary Occupancy Map
saveBinaryOccupancyMap(panoramicImage, filtered_stats, 'binary_occupancy_map.png');

%% Function to Create and Save Binary Occupancy Map
function saveBinaryOccupancyMap(panoramicImage, filtered_stats, filename)
    % Initialize a binary occupancy map with zeros (surface)
    binaryOccupancyMap = zeros(size(panoramicImage)); 

    % Fill the map with 1s (obstacles) where bounding boxes are detected
    for k = 1:length(filtered_stats)
        bb = filtered_stats(k).BoundingBox;
        % Create a binary mask for the current bounding box
        rectMask = false(size(panoramicImage));
        rectMask(round(bb(2)):(round(bb(2))+round(bb(4))-1), round(bb(1)):(round(bb(1))+round(bb(3))-1)) = true;

        % Update the occupancy map with obstacles (1)
        binaryOccupancyMap(rectMask) = 1;
    end

    % Save the binary occupancy map as an image file
    imwrite(binaryOccupancyMap, filename);

    % Display the binary occupancy map
    figure;
    imshow(binaryOccupancyMap, 'InitialMagnification', 'fit');
    colormap([1 1 1; 0 0 0]); % White for surface (0), black for obstacles (1)
    title('Binary Occupancy Map with Obstacles');
    disp(['Binary occupancy map saved as: ' filename]);
end