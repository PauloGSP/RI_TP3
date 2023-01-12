load('cameraParams.mat','cameraParams')
% Rectify the image
I = imread('checkcolor.jpg');
Irect = undistortImage(I, cameraParams);
imagePoints = detectCheckerboardPoints(I,PartialDetections=true);
[rows, columns, numberOfColorChannels] = size(Irect);
%estimate Extrinsics
camExtrinsics = estimateExtrinsics(imagePoints,cameraParams.WorldPoints,cameraParams.Intrinsics);
%i is y k is x
for i = 1:rows
    for k= 1:columns
        %para cada pixel vamos ler o RGB
    	r = Irect(i, k , 1);
        b = Irect(i, k , 3);
        g = Irect(i, k , 2);
        %Retirar as bordas laterais pra ter menos objetos na frente
        if k<260 || k>1075
            Irect(i, k , 1)=0;
            Irect(i, k , 2)=0;
            Irect(i, k , 3)=0;        
        end
        %retirar a borda de cima pra ter menos objetos a interferir
        if i<175
            Irect(i, k , 1)=0;
            Irect(i, k , 2)=0;
            Irect(i, k , 3)=0;
        end
        %colocar tudo q seja relativamente neutro (brancos pretos e cinzas)
        %em preto perfeito ([0 0 0]) para não haver conflitos
        if (r>77 && b>80 && g>80)
            Irect(i, k , 1)=0;
            Irect(i, k , 2)=0;
            Irect(i, k , 3)=0;
        elseif (r<80 && b<80 && g<80)
            Irect(i, k , 1)=0;
            Irect(i, k , 2)=0;
            Irect(i, k , 3)=0;
        
        end      
    end
    
end


%este for simplesmente intensifica as cores vermelhas e azuis e remove
%todas as outras
for i = 1:rows
    for k= 1:columns

        r = Irect(i, k , 1);
        b = Irect(i, k , 3);

        if r>110 && r<180
            Irect(i, k , 1)=255;
            Irect(i, k , 2)=0;
            Irect(i, k , 3)=0;
        elseif b>100
            Irect(i, k , 3)=255;

            Irect(i, k , 2)=0;
            Irect(i, k , 1)=0;
        else
            Irect(i, k , 3)=0;
            Irect(i, k , 2)=0;
            Irect(i, k , 1)=0; 
        end

        
        
    end
    
end



Iblue = imbinarize(Irect(:,:,3),'adaptive');
Ired = imbinarize(Irect(:,:,1),'adaptive');
% Use regionprops to extract the properties of the regions in the binary image
blueProps = regionprops(Iblue,'Centroid','Orientation');
redProps = regionprops(Ired,'Centroid','Orientation');
% For each blue object, calculate the position in the robot's frame using the camera's extrinsic parameters
for i = 1:length(blueProps)
    
    position = [blueProps(i).Centroid, 0];
    positionInRobotFrame = img2world2d(imagePoints,camExtrinsics,cameraParams.Intrinsics);
    
    bluePositions{i} = positionInRobotFrame;
end
positionInRobotFrame2=zeros(length(redProps),1);
% For each red object, calculate the position in the robot's frame using the camera's extrinsic parameters
for i = 1:length(redProps)
    position = [redProps(i).Centroid, 0];
    positionInRobotFrame2 = img2world2d(imagePoints,camExtrinsics,cameraParams.Intrinsics);
    redPositions{i} = positionInRobotFrame2;
end

%Visualize Original Image
imshow(I)
hold on

%estes 2 fors defininem as bases e as peças em 2 grupos diferentes
blueBaseCentroids=[];
redBaseCentroids=[];
bluePieceCentroids=[];
redPieceCentroids=[];
for i=1:length(blueProps)
    pos = blueProps(i).Centroid;
    pos = [pos(1), pos(2)];
    if pos(1)<445
        blueBaseCentroids= [blueBaseCentroids; pos];
    else
        bluePieceCentroids=[bluePieceCentroids;pos];
    end
end
for i=1:length(redProps)
    pos = redProps(i).Centroid;
    pos = [pos(1), pos(2)];
    if pos(1)>915
        redBaseCentroids= [redBaseCentroids; pos];
    else
        redPieceCentroids=[redPieceCentroids;pos];
    
    end
end

% Visualize the centroids on the image
% piece centroids should be in the right color 
% bases should be green for red team and yellow for blue team 
for i = 1:length(redPieceCentroids)
    pos = redPieceCentroids(i,:);
    pos = [pos(1), pos(2)];
    rectangle('Position',[pos(1)-10, pos(2)-10, 20, 20],'EdgeColor','r')
end
for i = 1:length(bluePieceCentroids)
    pos = bluePieceCentroids(i,:);
    pos = [pos(1), pos(2)];
    rectangle('Position',[pos(1)-10, pos(2)-10, 20, 20],'EdgeColor','b')
end

for i = 1:length(redBaseCentroids)
    pos = redBaseCentroids(i,:);
    pos = [pos(1), pos(2)];
    rectangle('Position',[pos(1)-10, pos(2)-10, 20, 20],'EdgeColor','g')
end
for i = 1:length(blueBaseCentroids)
    pos = blueBaseCentroids(i,:);
    pos = [pos(1), pos(2)];
    rectangle('Position',[pos(1)-10, pos(2)-10, 20, 20],'EdgeColor','y')
end
hold off

%missing work is to find the best centroid or just to correlate the
%coordinates from pixels to robot values for exemple we know that the
%top left corner of the checkboard is 560 -125 in robot coordinates and
%it is 513 466 and we also know that the folloing square(in the Y axis) 
% is 560 -101 for the robot and it is   545 466 in pixels

%this means our axis are swaped and 4/3 should be the resolution meaning 
% every 4 pixels are 3 mm

