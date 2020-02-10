clear vars;
close all;
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
sensorLocation  = [0, 0, 0]; % Sensor is at the center of the coordinate system

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

% while(hasFrame(data) && player.isOpen() && (data.CurrentTime < data.CurrentTime + seconds(10)))
    %% POINT CLOUD CRATION
    ptCloudTemp = readFrame(data);
    points = struct();
    
    % TODO: change radius
    radius          = 2; % meters
    nearbyPoints = findNeighborsInRadius(ptCloudTemp, ...
        sensorLocation, radius);                                            %cerco i punti vicini
    ptCloudObj = select(ptCloudTemp, nearbyPoints , 'OutputSize', 'full');  %restringo area ricerca per qualsiasi cosa
    
    %% POINTS GROUPING - CAR
    points.EgoPoints = helperSegmentEgoFromLidarData(ptCloudObj, vehicleDims, mountLocation);
    closePlayer = false;

    %% POINTS GROUPING - GROUND
    elevationDelta = 5;
    points.GroundPoints = segmentGroundFromLidarData(ptCloudObj, 'ElevationAngleDelta', elevationDelta);

    %% POINTS GROUPING - OBSTACLES
    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
%    nonEgoGroundPoints = ~points.GroundPoints;
    ptCloudSegmented = select(ptCloudObj, nonEgoGroundPoints, 'OutputSize', 'full');

    

    points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, ...
        sensorLocation, radius);


    %% CLASS TRY

    boxes = getBoundingBoxes(ptCloudSegmented, 0.1, 20, 0.5, -0.5);
    
    %% VISUALIZZAZIONE
    % Visualize the segmented obstacles
    [a,b,c]=size(ptCloudObj.Location);
    prova=ptCloudObj.Location;
    for x = 1:a
        for y = 2:b
            for z = 1:c
                prova(x,y,z)=0;
            end
        end
    end
    %TODO: Render boxes

    helperUpdateView(player, ptCloudTemp, points, colors, closePlayer);

p=ptCloudTemp.Location;
x=p(:,:,1);
y=p(:,:,2);
z=p(:,:,3);
[a,b]=size(x);
for i=1:a
    for j=1:b
        if y(i,j)>3.3 || y(i,j)<-2.5 || x(i,j)<-4.3 || x(i,j)>1.8
            x(i,j)=nan;
            y(i,j)=nan;
            z(i,j)=nan;
        end
    end
end
plot3(x,y,z,'.','Markersize',1,'Color','[0 0.4470 0.7410]');
[s1,s2]=size(boxes);
hold on;
for k=1:s2
x1=[boxes(1,k),boxes(1,k)];
y1=[boxes(2,k),boxes(2,k)];
z1=[boxes(3,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(1,k),boxes(1,k)];
y1=[boxes(5,k),boxes(5,k)];
z1=[boxes(3,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(4,k),boxes(4,k)];
y1=[boxes(2,k),boxes(2,k)];
z1=[boxes(3,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(4,k),boxes(4,k)];
y1=[boxes(5,k),boxes(5,k)];
z1=[boxes(3,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');

x1=[boxes(1,k),boxes(4,k)];
y1=[boxes(2,k),boxes(2,k)];
z1=[boxes(3,k),boxes(3,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(1,k),boxes(4,k)];
y1=[boxes(2,k),boxes(2,k)];
z1=[boxes(6,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(1,k),boxes(4,k)];
y1=[boxes(5,k),boxes(5,k)];
z1=[boxes(3,k),boxes(3,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(1,k),boxes(4,k)];
y1=[boxes(5,k),boxes(5,k)];
z1=[boxes(6,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');

x1=[boxes(1,k),boxes(1,k)];
y1=[boxes(2,k),boxes(5,k)];
z1=[boxes(3,k),boxes(3,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(1,k),boxes(1,k)];
y1=[boxes(2,k),boxes(5,k)];
z1=[boxes(6,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(4,k),boxes(4,k)];
y1=[boxes(2,k),boxes(5,k)];
z1=[boxes(3,k),boxes(3,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
x1=[boxes(4,k),boxes(4,k)];
y1=[boxes(2,k),boxes(5,k)];
z1=[boxes(6,k),boxes(6,k)];
plot3(x1,y1,z1,'Color','[0.6350 0.0780 0.1840]');
end
%     pause(0.1);
% end

function bboxes = getBoundingBoxes(ptCloud,minDistance,minDetsPerCluster,maxZDistance,minZDistance)
    % This method fits bounding boxes on each cluster with some basic
    % rules.
    % Cluster must have atleast minDetsPerCluster points.
    % Its mean z must be between maxZDistance and minZDistance.
    % length, width and height are calculated using min and max from each
    % dimension.
    [labels,numClusters] = pcsegdist(ptCloud,minDistance);
    pointData = ptCloud.Location;
    bboxes = nan(6,numClusters,'like',pointData);
    isValidCluster = false(1,numClusters);
    for i = 1:numClusters
        %dimensioni della matrice di partenza
        [a,b,~]=size(pointData);
        t=0;
        %inizializzo le tre matrici di appoggio
        thisX=nan(a,b);
        thisY=nan(a,b);
        thisZ=nan(a,b);
        %popolo le matrici di appoggio con i dati dell'i-esimo cluster
        for x=1:a
           for y=1:b
               if labels(x,y)==i
               thisX(x,y)=pointData(x,y,1);     %matrice delle x dei punti del cluster 
               thisY(x,y)=pointData(x,y,2);     %matrice delle y dei punti del cluster
               thisZ(x,y)=pointData(x,y,3);     %matrice delle z dei punti del cluster
               t=t+1;                           %numero punti del cluster
               else
                   thisX(x,y)=nan;
                   thisY(x,y)=nan;
                   thisZ(x,y)=nan;
               end
           end
        end        
        meanPoint = max(max(thisZ))-min(min(thisZ));
        if t > minDetsPerCluster && ...
                meanPoint < maxZDistance && meanPoint > minZDistance
            xMin = min(min(thisX));
            xMax = max(max(thisX));
            yMin = min(min(thisY));
            yMax = max(max(thisY));
            zMin = min(min(thisZ));
            zMax = max(max(thisZ));
             l = (xMax - xMin);
%             w = (yMax - yMin);
%             h = (zMax - zMin);
%             x = (xMin + xMax)/2;
%             y = (yMin + yMax)/2;
%             z = (zMin + zMax)/2;
            bboxes(:,i) = [xMin yMin zMin xMax yMax zMax]';
            isValidCluster(i) = l < 20; % max length of 20 meters
        end
    end
    bboxes = bboxes(:,isValidCluster);
end

