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
    radius          = 2.3; % meters
    nearbyPoints = findNeighborsInRadius(ptCloudTemp, ...
        sensorLocation, radius);                                            %cerco i punti vicini
    ptCloudObj = select(ptCloudTemp, nearbyPoints , 'OutputSize', 'full');  %restringo area ricerca per qualsiasi cosa
    
    %% POINTS GROUPING - CAR
    zlidar=0.12;                                                            %posizione relativa del lidar rispetto alla scrivania
    points.EgoPoints = helperSegmentEgoFromLidarData(ptCloudObj, vehicleDims, mountLocation,zlidar);
    closePlayer = false;

    %% POINTS GROUPING - GROUND
    elevationDelta = 5;
    points.GroundPoints = segmentGroundFromLidarData(ptCloudObj, 'ElevationAngleDelta', elevationDelta);

    %% POINTS GROUPING - OBSTACLES
    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
%     nonEgoGroundPoints = ~points.GroundPoints;
    ptCloudSegmented = select(ptCloudObj, nonEgoGroundPoints, 'OutputSize', 'full');

    

    points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, ...
        sensorLocation, radius);


    %% CLASS TRY
    tic
    boxes = getBoundingBoxes(ptCloudSegmented, 0.1, 20, 0.5, -0.5);
    toc
    boxes2 = getBoundingBoxes2(ptCloudSegmented, 20);
    toc
    %% VISUALIZZAZIONE
    % Visualize the segmented obstacles
%     [a,b,c]=size(ptCloudObj.Location);
%     prova=ptCloudObj.Location;
%     for x = 1:a
%         for y = 2:b
%             for z = 1:c
%                 prova(x,y,z)=0;
%             end
%         end
%     end
    
    % Visualize segmented parts
    helperUpdateView(player, ptCloudTemp, points, colors, closePlayer);
    
    % Visualize boxes
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

    %plotting boxes
%     for k=1:s2
%         X = [boxes(1,k),boxes(4,k),boxes(4,k),boxes(1,k),boxes(1,k)];
%         Y = [boxes(2,k),boxes(2,k),boxes(5,k),boxes(5,k),boxes(2,k)];
%         Z1 = [boxes(3,k),boxes(3,k),boxes(3,k),boxes(3,k),boxes(3,k)];
%         Z2 = [boxes(6,k),boxes(6,k),boxes(6,k),boxes(6,k),boxes(6,k)];
%         plot3(X,Y,Z1,'Color','[0.6350 0.0780 0.1840]');
%         plot3(X,Y,Z2,'Color','[0.6350 0.0780 0.1840]');
%         plot3([X(1:4);X(1:4)],[Y(1:4);Y(1:4)],[Z1(1);Z2(1)],'Color','[0.6350 0.0780 0.1840]');
% 
%     end

    [b1,b2]=size(boxes2);
    for k=1:b2
        X = [boxes2(1,k),boxes2(4,k),boxes2(4,k),boxes2(1,k),boxes2(1,k)];
        Y = [boxes2(2,k),boxes2(2,k),boxes2(5,k),boxes2(5,k),boxes2(2,k)];
        Z1 = [boxes2(3,k),boxes2(3,k),boxes2(3,k),boxes2(3,k),boxes2(3,k)];
        Z2 = [boxes2(6,k),boxes2(6,k),boxes2(6,k),boxes2(6,k),boxes2(6,k)];
        plot3(X,Y,Z1,'Color','[0.6350 0.0780 0.1840]');
        plot3(X,Y,Z2,'Color','[0.6350 0.0780 0.1840]');
        plot3([X(1:4);X(1:4)],[Y(1:4);Y(1:4)],[Z1(1);Z2(1)],'Color','[0.6350 0.0780 0.1840]');

    end

%     pause(0.1);
% end