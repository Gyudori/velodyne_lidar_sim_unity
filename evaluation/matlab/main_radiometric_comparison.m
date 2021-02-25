clc; clear all; close all


%% Preprocessing

% Load real lidar data
veloReader = velodyneFileReader('./data/2019-09-27-13-37-39_Velodyne-VLP-16-Data_600rpm_mid_strong.pcap', 'VLP16');
realPtCloud = readFrame(veloReader);

% Load simulated lidar data
datapath = '../../output_sample/';
basicData = readmatrix([datapath,'Radiometric/16Channel__First_Frame.csv'], 'Delimiter', ',');
basicData = basicData(basicData(:, 4) ~= 0, :); % Delete empty rows
basicPtCloud =  pointCloud([basicData(:, 3), basicData(:, 5), basicData(:, 4)], 'Intensity', basicData(:, 6));

preciseData = readmatrix([datapath, 'Radiometric/16Channel__First_Frame_Precise.csv'], 'Delimiter', ',');
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

% Domain matching
realMean = mean(realPtCloudROI.Intensity);
realStd = std(cast(realPtCloudROI.Intensity, 'double'));
basicMean = mean(basicPtCloudROI.Intensity);
basicStd = std(cast(basicPtCloudROI.Intensity, 'double'));

basicPtCloudROI.Intensity = (basicPtCloudROI.Intensity - basicMean) / basicStd * realStd + realMean;


%% Intensity 

% Calculate intensity difference of nearest point
indices = zeros(realPtCloudROI.Count, 1);
basicDiff = zeros(realPtCloudROI.Count, 1);
preciseDiff = zeros(realPtCloudROI.Count, 1);

for i = 1:realPtCloudROI.Count
    [indices(i),dists] = findNearestNeighbors(basicPtCloudROI,realPtCloudROI.Location(i, :), 1);
    basicDiff(i) = abs(double(basicPtCloudROI.Intensity(indices(i))) - double(realPtCloudROI.Intensity(i)));
    preciseDiff(i) = abs(double(precisePtCloudROI.Intensity(indices(i))) - double(realPtCloudROI.Intensity(i)));
end


%% Plotting
% % Segmented point cloud (Real data/Sim data)
% figure('Position', [100 100 700 400]);
% subplot(1, 2, 1); pcshow(realPtCloudROI); title('Real point cloud'); cax = caxis; xlabel('X'); ylabel('Y'); zlabel('Z');
% subplot(1, 2, 2); pcshow(simPtCloudROI); title('Sim point cloud'); caxis(cax); xlabel('X'); ylabel('Y'); zlabel('Z');

% Intensity Plotting
figure('Position', [100 100 1200 400]);
subplot(1, 3, 1); pcshow(realPtCloudROI); title('Real'); cax = caxis;colorbar
subplot(1, 3, 2); pcshow(basicPtCloudROI); title('Basic'); caxis(cax);colorbar
subplot(1, 3, 3); pcshow(precisePtCloudROI); title('Precise'); caxis(cax); colorbar;

% Histogram
numOfBins = 30;
figure('Position', [100 100 600 200]); hold on;
subplot(1, 3, 1); histogram(realPtCloudROI.Intensity, numOfBins); title('Real'); xlim([0, 150]); ylim([0, 4000]); xlabel('Intensity');
subplot(1, 3, 2); histogram(basicPtCloudROI.Intensity, numOfBins); title('Basic'); xlim([0, 150]); ylim([0, 4000]); xlabel('Intensity');
subplot(1, 3, 3); histogram(precisePtCloudROI.Intensity, numOfBins); title('Precise'); xlim([0, 150]); ylim([0, 4000]); xlabel('Intensity');

% Plot difference of intensity
basicDiffPtCloud = pointCloud(realPtCloudROI.Location, 'Intensity', basicDiff);
preciseDiffPtCloud = pointCloud(realPtCloudROI.Location, 'Intensity', preciseDiff);
figure('Position', [100 100 600 250]); 
subplot(1, 2, 1); pcshow(basicDiffPtCloud); title('| Real - Basic |'); caxis([0 100]); 
subplot(1, 2, 2); pcshow(preciseDiffPtCloud); title('| Real - Precise |'); colorbar; caxis([0 100]); 

% Histogram
numOfBins = 50;
figure('Position', [100 100 600 250]); 
subplot(1, 2, 1); histogram(basicDiff, numOfBins); 
title('| Real - Basic |'); xlim([0, 100]); ylim([0, 6000]); xlabel('Intensity difference');
subplot(1, 2, 2); histogram(preciseDiff, numOfBins); 
title('| Real - Precise |'); xlim([0, 100]); ylim([0, 6000]); xlabel('Intensity difference');


% Graph 
figure; hold on;
plot(basicDiff, '.'); plot(preciseDiff, '.'); 














