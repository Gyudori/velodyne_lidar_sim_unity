clc; clear all; close all;


%% Load data
veloReader = velodyneFileReader('./input data/2019-09-27-13-37-39_Velodyne-VLP-16-Data_600rpm_mid_strong.pcap', 'VLP16');
realPtCloud = readFrame(veloReader);

% realData = readmatrix('./input data/frame1_segmented.csv', 'Delimiter', ',');
% realPtCloud = pointCloud(realData(:, 1:3), 'Intensity', realData(:, 7));

% pcshow(realPtCloud);

simData = readmatrix('./input data/277.csv', 'Delimiter', ',');
simData = simData(simData(:, 4) ~= 0, :); % Delete empty rows
simPtCloud =  pointCloud([simData(:, 2), simData(:, 4), simData(:, 3)], 'Intensity', simData(:, 5)*100);

% % plotting
% hold on
% pcshow(simPtCloud);
% pcshow(realPtCloud);
% xlabel('X'); ylabel('Y'); zlabel('Z');


%% Segment point clouds in ROI

ROI = [-inf inf -2.8 inf -inf inf];

indices = findPointsInROI(realPtCloud, ROI);
realPtCloudROI = select(realPtCloud, indices, 'OutputSize', 'full');

indices = findPointsInROI(simPtCloud, ROI);
simPtCloudROI = select(simPtCloud, indices);

clear ROI;

%% Segment planes

ROI.left = [-inf 0 -inf inf -inf inf];
ROI.front = [-inf inf 0 inf -inf inf];
ROI.right = [0 inf, -inf, inf, -inf, inf];
ROI.bottom = [-inf inf -inf inf -10 0];

referenceVector.left = [1,0,0];
referenceVector.front = [0,1,0];
referenceVector.right = [1,0,0];
referenceVector.bottom = [0,0,1];

% Segment 4 planes
planePtCloud = SegmentAllPlanes(realPtCloud, ROI, referenceVector);

% % Plotting
% for i = 1:4
%     figure; pcshow(planePtCloud(i).ptCloud);
% end

%% Correct intensity of point cloud on plane with function (CorrectIntensityPlanePtCloud)

ptCloudReflectance = pointCloud.empty(length(planePtCloud), 0);
for i = 1:length(planePtCloud)
    ptCloudReflectance(i) = RangeEquatinoCorrection(planePtCloud(i));
end

%% Plotting


% for i = 1:length(planePtCloud)    
%     figure;
%     subplot(2, 2, 1); pcshow(planePtCloud(i).ptCloud); title('Original Intensity');
%     subplot(2, 2, 2); pcshow(ptCloudReflectance(i)); title('Corrected Intensity(Reflectance)');    
% 
%     subplot(2, 2, 3); histogram(planePtCloud(i).ptCloud.Intensity); title('Original Intensity');
%     subplot(2, 2, 4); histogram(ptCloudReflectance(i).Intensity); title('Corrected Intensity(Reflectance)');
%     set(gcf,'Color','w', 'Position', [100, 100, 1000, 500])
% end


%% Noise elimination
ptCloudRefined = pointCloud.empty(length(planePtCloud), 0);
ptCloudOrig = pointCloud.empty(length(planePtCloud), 0);
for i = 1:length(planePtCloud)
    
    % intensity value of point cloud
    originalIntensity = planePtCloud(i).ptCloud.Intensity;
    correctedIntensity = ptCloudReflectance(i).Intensity;
    
    % NaN exception
    numIndex = ~isnan(correctedIntensity);   
    
    correctedIntensity = correctedIntensity(numIndex);
    correctedPoints = reshape(ptCloudReflectance(i).Location, [], 3);
    correctedPoints = correctedPoints(reshape(numIndex, [], 1), :);    
    
    originalIntensity = originalIntensity(numIndex);
    originalPoints = reshape(planePtCloud(i).ptCloud.Location, [], 3);
    originalPoints = originalPoints(reshape(numIndex, [], 1), :);    
    
    
    % Boundary of inlier
    min = mean(correctedIntensity) - 2*std(correctedIntensity);
    max = mean(correctedIntensity) + 2*std(correctedIntensity);  
    inlierIndex = min < correctedIntensity &  correctedIntensity < max;    
    
    % extract inlier
    correctedIntensity = correctedIntensity(inlierIndex);
    correctedPoints = correctedPoints(inlierIndex, :);
    
    originalIntensity = originalIntensity(inlierIndex);
    originalPoints = originalPoints(inlierIndex, :);
    
    % Plotting
    ptCloudRefined(i) = pointCloud(correctedPoints, 'Intensity', correctedIntensity);
    ptCloudOrig(i) = pointCloud(originalPoints, 'Intensity', originalIntensity);
    
    figure; 
    subplot(1, 2, 1); pcshow(ptCloudOrig(i)); title('Original');
    subplot(1, 2, 2); pcshow(ptCloudRefined(i)); title('Refined');
end


%% Compute final reflectance

% planePtCloud : original point cloud intensity
% ptCloudReflectance : corrected reflectance with noise
% ptCloudRefined : noise eliminated reflectance

for i = 1:length(planePtCloud)
    
    % Intensity values
    original = reshape(planePtCloud(i).ptCloud.Intensity, [], 1);
    original = original(~isnan(original));
    
    correctedIntensity = reshape(ptCloudReflectance(i).Intensity, [], 1);
    correctedIntensity = correctedIntensity(~isnan(correctedIntensity));    
    
    refined = ptCloudRefined(i).Intensity;
    
    % Compute median to normalize distribution    
    originalMedian = median(original);
    correctedMedian = median(correctedIntensity);
    refinedMedian = median(ptCloudRefined(i).Intensity);
    
    figure;
    subplot(1, 3, 1); histogram(planePtCloud(i).ptCloud.Intensity); title('Original');
    subplot(1, 3, 2); histogram(correctedIntensity * originalMedian / correctedMedian); title('corrected reflectance');
    subplot(1, 3, 3); histogram(refined * originalMedian / refinedMedian); title('refined reflectance');
end


    










