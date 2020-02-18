function bboxes = getBoundingBoxes(ptCloud,minDistance,minDetsPerCluster,maxZDistance,minZDistance)
    % Questo metodo crea "bounding boxes" per ogni cluster con delle
    % semplici regole
    % Cluster devono avere almeno minDetsPerCluster punti.
    % La media di z deve essere tra maxZDistance e minZDistance.
    % length, width e height sono calcolate usando min e max da ogni
    % dimensione
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
            isValidCluster(i) = l < 20; % max length in metri
        end
    end
    bboxes = bboxes(:,isValidCluster);
end