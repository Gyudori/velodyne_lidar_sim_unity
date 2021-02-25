clc; clear all; close all


%% Preprocessing

% Load real lidar data
veloReader = velodyneFileReader('./data/2019-09-27-13-37-39_Velodyne-VLP-16-Data_600rpm_mid_strong.pcap', 'VLP16');
realPtCloud = readFrame(veloReader);

% Load simulated lidar data
datapath = '../../output_sample/';
basicData = readmatrix([datapath,'Geometric/16Channel__First_Frame.csv'], 'Delimiter', ',');
basicData = basicData(basicData(:, 4) ~= 0, :); % Delete empty rows
basicPtCloud =  pointCloud([basicData(:, 3), basicData(:, 5), basicData(:, 4)], 'Intensity', basicData(:, 6));

preciseData = readmatrix([datapath, 'Geometric/16Channel__First_Frame_Precise.csv'], 'Delimiter', ',');
preciseData = preciseData(preciseData(:, 4) ~= 0, :); % Delete empty rows
precisePtCloud =  pointCloud([preciseData(:, 3), preciseData(:, 5), preciseData(:, 4)], 'Intensity', preciseData(:, 6));

% Segment point clouds in ROI
ROI = [-inf inf -2.8 inf -inf inf];
indices = findPointsInROI(realPtCloud, ROI);
realPtCloudROI = select(realPtCloud, indices);

indices = findPointsInROI(basicPtCloud, ROI);
basicPtCloudROI = select(basicPtCloud, indices);

indices = findPointsInROI(precisePtCloud, ROI);
precisePtCloudROI = select(precisePtCloud, indices);

% plotting original point cloud
% figure; pcshow(realPtCloud); title('Original real LIDAR point cloud');
% xlabel('X'); ylabel('Y'); zlabel('Z');

%% Calculate distance

% Calculate distance of nearest point
basicDist = zeros(realPtCloudROI.Count, 1);
preciseDist = zeros(realPtCloudROI.Count, 1);
for i = 1:realPtCloudROI.Count
    [indices,basicDist(i)] = findNearestNeighbors(basicPtCloudROI, realPtCloudROI.Location(i, :),1);
    [indices,preciseDist(i)] = findNearestNeighbors(precisePtCloudROI, realPtCloudROI.Location(i, :),1);
end

%% Position Plotting

% Overlayed point cloud
figure; hold on;
pointSize = 100;
pcshow(realPtCloudROI.Location, 'w', 'MarkerSize', pointSize);
pcshow(precisePtCloudROI.Location, 'g', 'MarkerSize', pointSize);
pcshow(basicPtCloudROI.Location, 'r', 'MarkerSize', pointSize);

% Distance preprocessing
[sortedPreciseDist, sortIndex] = sort(preciseDist, 'descend');
outlierSize = round(length(preciseDist)*0.001);
realPtCloudROI_temp = realPtCloudROI.Location(sortIndex, :);
inlierRealPtCloudROI = realPtCloudROI_temp(outlierSize+1:end, :);

tempBasicDist = basicDist(sortIndex);
inlierBasicDist = tempBasicDist(outlierSize+1:end);
inlierPreciseDist = sortedPreciseDist(outlierSize+1:end);

% Plotting
basicPtCloudROIDist = pointCloud(inlierRealPtCloudROI, 'Intensity', inlierBasicDist);
precisePtCloudROIDist = pointCloud(inlierRealPtCloudROI, 'Intensity', inlierPreciseDist);
figure('Position', [100 100 1200 400]); 
subplot(1, 2, 1); pcshow(basicPtCloudROIDist, 'MarkerSize', pointSize); title('Real, Basic Distance'); cax = caxis; 
subplot(1, 2, 2); pcshow(precisePtCloudROIDist, 'MarkerSize', pointSize); title('Real, Precise Distance'); colorbar; caxis(cax);

% Histogram
numOfBins = 50;
figure('Position', [100 100 1200 400]); 
subplot(1, 2, 1); histogram(inlierBasicDist, numOfBins); xlim([-0.01, 0.2]); title('Real, Basic'); xlabel('Distance (m)');
subplot(1, 2, 2); histogram(inlierPreciseDist, numOfBins); xlim([-0.01, 0.2]); title('Real, Precise'); xlabel('Distance (m)');









