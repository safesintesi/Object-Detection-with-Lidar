close all
clear

%% data loading
dataMainDir = './';
configID = '2';
fullFolderPath = fullfile(dataMainDir,sprintf('/Config%s',configID));
fileList = dir(fullFolderPath);
nameList = {fileList.name};
nameList = nameList(3:end);
% fileDateTime = '2019-12-12-15-07-21';
fileName = nameList{endsWith(nameList,'_Velodyne-VLP-16-Data.pcap')};
fullFilePath = fullfile(fullFolderPath,fileName);

fileList = dir(fullFilePath);

deviceModel = 'VLP16';

data = velodyneFileReader(fullFilePath,deviceModel);

%% CREO LABLES PER COLORI
% Define labels to use for segmented points
colorLabels = [...
    0      0.4470 0.7410; ... % Unlabeled points, specified as [R,G,B]
    0.4660 0.6740 0.1880; ... % Ground points
    0.9290 0.6940 0.1250; ... % Ego points
    0.6350 0.0780 0.1840];    % Obstacle points

% Define indices for each label
colors.Unlabeled = 1;
colors.Ground    = 2;
colors.Ego       = 3;
colors.Obstacle  = 4;

%% divido i punti
points = struct();


%% create player
% xlimits = [-0.5 0];
% ylimits = [0.65 1];
% zlimits = [-0.2 0.2];
xlimits = [-5 3];
ylimits = [-2.6 4];
zlimits = [-2 2];


player = pcplayer(xlimits,ylimits,zlimits);
xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');
minDistance = 0.1;
data.CurrentTime = data.EndTime - seconds(10);
% while(hasFrame(data) && player.isOpen() && (data.CurrentTime < data.CurrentTime + seconds(10)))
%     ptCloudObj = readFrame(data);
% %     ptCloudSeg = pcsegdist(ptCloudObj,minDistance);
% %     pcshow(ptCloudObj.Location,ptCloudSeg);
% %     view(player,ptCloudObj.Location,ptCloudSeg);
%     view(player,ptCloudObj.Location,ptCloudObj.Intensity);
%     pause(10);
% end

% Set the colormap
colormap(player.Axes, colorLabels)

ptCloudObj= readFrame(data);
view(player,ptCloudObj.Location);