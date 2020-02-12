function bboxes2 = getBoundingBoxes2(ptCloud,minDetsPerCluster)
    prova=ptCloud.Location;
    [p1,p2,~]=size(prova);
    punti=0;
    for i=1:p1
       for j=1:p2
          if ~isnan(prova(i,j,1))
             punti=punti+1;
          end
       end
    end
    data=zeros(punti,3);
    punti=1;
    i=0;
    j=0;
    for i=1:p1
       for j=1:p2
          if ~isnan(prova(i,j,1))
             data(punti,1)=prova(i,j,1);
             data(punti,2)=prova(i,j,2);
             data(punti,3)=prova(i,j,3);
             punti=punti+1;
          end
       end
    end
    numClusters=20;
    [labels] = kmeans(data,numClusters);
    bboxes2 = nan(6,numClusters);
    isValidCluster = false(1,numClusters);
    punti=punti-1;
    for i = 1:numClusters        
        t=0;
        %inizializzo le tre matrici di appoggio
        thisX=nan(punti,1);
        thisY=nan(punti,1);
        thisZ=nan(punti,1);
        %popolo le matrici di appoggio con i dati dell'i-esimo cluster
        for x=1:punti
               if labels(x)==i
               thisX(x)=data(x,1);     %matrice delle x dei punti del cluster 
               thisY(x)=data(x,2);     %matrice delle y dei punti del cluster
               thisZ(x)=data(x,3);     %matrice delle z dei punti del cluster
               t=t+1;                           %numero punti del cluster
               else
                   thisX(x)=nan;
                   thisY(x)=nan;
                   thisZ(x)=nan;
               end
        end
        if t > minDetsPerCluster
            xMin = min(thisX);
            xMax = max(thisX);
            yMin = min(thisY);
            yMax = max(thisY);
            zMin = min(thisZ);
            zMax = max(thisZ);
             l = (xMax - xMin);
%             w = (yMax - yMin);
%             h = (zMax - zMin);
%             x = (xMin + xMax)/2;
%             y = (yMin + yMax)/2;
%             z = (zMin + zMax)/2;
            bboxes2(:,i) = [xMin yMin zMin xMax yMax zMax]';
            isValidCluster(i) = l < 20; % max length of 20 meters
        end
    end
    bboxes2 = bboxes2(:,isValidCluster);
end