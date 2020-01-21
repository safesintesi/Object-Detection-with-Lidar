%% DATA LOADING
dataMainDir = './';
configID = '2';
fullFolderPath = fullfile(dataMainDir,sprintf('/../Config%s',configID));
fileList = dir(fullFolderPath);
nameList = {fileList.name};
nameList = nameList(3:end);
% fileDateTime = '2019-12-12-15-07-21';
fileName = nameList{endsWith(nameList,'_Velodyne-VLP-16-Data.pcap')};
fullFilePath = fullfile(fullFolderPath,fileName);

fileList = dir(fullFilePath);

deviceModel = 'VLP16';

data = velodyneFileReader(fullFilePath,deviceModel);

%% COLOR LABELS
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

%% VEHICLE CREATION
% TODO:
% MUST CHANGE WITH VEHICLE DIMENSIONS (length, width, height)
length = 1.5;
width = 2.2;
height = 1;
vehicleDims = vehicleDimensions(length, width, height); % Typical vehicle 4.7m by 1.8m by 1.4m

% lidar's position relative to the vehicle
mountLocation = [...
    vehicleDims.Length/2 - vehicleDims.RearOverhang, ... % x
    0, ...                                               % y
    vehicleDims.Height];                                 % z

%% PLAYER
%room
% xlimits = [-5 3];
% ylimits = [-4 5];
% zlimits = [-2 2];

%object
% xlimits = [-0.5 0];
% ylimits = [0.65 1];
% zlimits = [-0.2 0.2];

%room no walls
xlimits = [-4.3 1.8];
ylimits = [-2.5 3.3];
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

%% POINTS GROUPING - CAR
ptCloudObj= readFrame(data);
points = struct();
points.EgoPoints = helperSegmentEgoFromLidarData(ptCloudObj, vehicleDims, mountLocation);

% view(player,ptCloudObj.Location);
% rendering "car"
closePlayer = false;
helperUpdateView(player, ptCloudObj, points, colors, closePlayer);

%% POINTS GROUPING - GROUND
elevationDelta = 5;
points.GroundPoints = segmentGroundFromLidarData(ptCloudObj, 'ElevationAngleDelta', elevationDelta);

% Visualize the segmented ground plane.
helperUpdateView(player, ptCloudObj, points, colors, closePlayer);

%% POINTS GROUPING - OBSTACLES
nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
ptCloudSegmented = select(ptCloudObj, nonEgoGroundPoints, 'OutputSize', 'full');

sensorLocation  = [0, 0, 0]; % Sensor is at the center of the coordinate system
% TODO: change radius
radius          = 3; % meters

points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, ...
    sensorLocation, radius);

% Visualize the segmented obstacles
helperUpdateView(player, ptCloudObj, points, colors, closePlayer);