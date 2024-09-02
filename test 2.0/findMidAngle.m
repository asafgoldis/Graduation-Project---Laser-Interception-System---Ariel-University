%% find Mid Angle
function mid_Position = findMidAngle(X_servo, Y_servo, cam, videoPlayer,...
                                     x_guess_Position, y_guess_Position)

numOfPixels = 10;
sensitivity = 0.1;
%delayTime = 1.5;

frame = snapshot(cam);
frameSize = size(frame);

X_Position = x_guess_Position;
Y_Position = y_guess_Position;

X_servo.writePosition(X_Position)
Y_servo.writePosition(Y_Position)

X_ok = 0;
Y_ok = 0;
centroid = [0.0 0.0];
delta = 0.0001;

while (~Y_ok || ~X_ok)
    
    if (X_ok==0 && centroid(1)>frameSize(2)/2+3)
        X_Position = X_Position + delta;
        X_servo.writePosition(X_Position)
    elseif (X_ok==0 && centroid(1)<frameSize(2)/2-3)
        X_Position = X_Position - delta;
        X_servo.writePosition(X_Position)
    end
    
    if (Y_ok==0 && centroid(2)>frameSize(1)/2+3)
        Y_Position = Y_Position - delta;
        Y_servo.writePosition(Y_Position)
    elseif (Y_ok==0 && centroid(2)<frameSize(1)/2-3)
        Y_Position = Y_Position + delta;
        Y_servo.writePosition(Y_Position)   
    end
    
    
    %tic
    %while toc < delayTime
    %end    
    
    frame = snapshot(cam);
    subtract_im = imsubtract(frame(:,:,1), rgb2gray(frame));
    bin_im = imbinarize(subtract_im, sensitivity);
    clean_bin_im = bwareaopen(bin_im, numOfPixels);
    object = regionprops(clean_bin_im, 'BoundingBox', 'Centroid');
    
     % find the biggest object
    bbox = zeros(length(object),4);
    for object_num = 1:length(object)
        bbox(object_num,:) = object(object_num).BoundingBox;
    end
    
    if ~isempty(bbox)
        objectSize = bbox(:,3).*bbox(:,4);
        biggestObjectNum = objectSize()==max(objectSize);
        
        centroid = object(biggestObjectNum).Centroid;
        
        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');
        
        % Display center mass centroid. 
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);
        
        frame = insertText(frame, [centroid(1)+15 centroid(2)], centroid_text,'TextColor', 'black', 'FontSize', 12, 'BoxColor', 'red', 'BoxOpacity', 1);
        
        if (frameSize(2)/2-3<centroid(1) && centroid(1)<frameSize(2)/2+3)
            X_ok = 1;
        else
            X_ok = 0;
        end
        
        if (frameSize(1)/2-3<centroid(2) && centroid(2)<frameSize(1)/2+3)
            Y_ok = 1;
        else
            Y_ok = 0;
        end
    end
    centroid
    videoPlayer.step(frame);

end
centroid
mid_Position = [X_servo.readPosition Y_servo.readPosition]

%mid_angle = [95 79];

%{
%%

%% laserCalibration %%
function centroid2Angle = laserCalibration(...
    X_servo, Y_servo, cam, videoPlayer,...
    startAngle_X, stopAngle_X, startAngle_Y,stopAngle_Y)

% find general valou
sensitivity = 0.09;  % findBiggestObject
numOfPixels = 8;  % findBiggestObject
delayTime = 1.25;

% find corners
delta = 0.1/180;
cornerDelayTime = 0.09;
pixelsPrecision = 8;
mid_point = [82/180 70/180];
%%
disp('laser Calibration Mode.');
disp('serching for points... stege 1/5');
frame = snapshot(cam);
frameSize = size(frame);
angleMap = zeros( 180, 180, 2);

x_direction = 1;
for y = startAngle_Y:5:stopAngle_Y
    Y_servo.writePosition(y/180)
    
for X = startAngle_X:5:stopAngle_X
    if x_direction == 1; x = startAngle_X+stopAngle_X-X;
    else; x = X; end
    tic
    X_servo.writePosition(x/180)
     while toc < delayTime
     frame = snapshot(cam);
     end
     
    [isFound, centroid, frame] = findBiggestObject(frame, sensitivity, numOfPixels);
    
    if isFound
        angleMap(x,y, :) = centroid(:);
        for i = 1:1:10
        for j = 1:1:10
            angleMap(x+i ,y+j, :) = angleMap(x ,y, :);
        end
        end
    end
    
    videoPlayer.step(frame);
end
    if (x_direction == 1); x_direction = 0;
    else; x_direction = 1; end
end

%% find corners
% find Up Left Corner
disp('searching for the up left corner. stege 2/5')
X_servo.writePosition(0.5)
X_servo.writePosition(0.5)
[X_ok, Y_ok, x_angle, y_angle, centroid] = parametersReset(mid_point, frameSize);
while (~Y_ok || ~X_ok)
    if centroid(1)>= pixelsPrecision
        x_angle = x_angle + delta;
        if (x_angle > 1 || x_angle < 0); x_angle = mid_point(1); end
        X_servo.writePosition(x_angle)
    end
    if centroid(2)>= pixelsPrecision
        y_angle = y_angle - delta;
        if (y_angle > 1 || y_angle < 0); y_angle = mid_point(2); end
        Y_servo.writePosition(y_angle)
    end
    
    tic
    while toc < cornerDelayTime
    frame = snapshot(cam);
    end
    
    [isFound, centroid, frame] = findBiggestObject(frame, sensitivity, numOfPixels);
    
    if isFound
        angleMap(round(x_angle*180) ,round(y_angle*180), :) = centroid(:);
        

        for i = 1:1:10
        for j = 1:1:10
            angleMap(round(x_angle*180)+i ,round(y_angle*180)+j, :) =...
                    angleMap(round(x_angle*180) ,round(y_angle*180), :);
        end
        end
        
        
    if (centroid(1) <= pixelsPrecision); X_ok = 1; end
    if (centroid(2) <= pixelsPrecision); Y_ok = 1; end
    end
    
	videoPlayer.step(frame);
end
disp('up left corner has found!!!');

%%
% find Up Right Corner
disp('searching for the up right corner. stege 3/5')
X_servo.writePosition(0.5)
X_servo.writePosition(0.5)
[X_ok, Y_ok, x_angle, y_angle, centroid] = parametersReset(mid_point, frameSize);
while (~Y_ok || ~X_ok)
    if centroid(1) <= frameSize(2) - pixelsPrecision
        x_angle = x_angle - delta;
        if (x_angle > 1 || x_angle < 0); x_angle = mid_point(1); end
        X_servo.writePosition(x_angle)
    end
    if centroid(2) >= pixelsPrecision
        y_angle = y_angle - delta;
        if (y_angle > 1 || y_angle < 0); y_angle = mid_point(2); end
        Y_servo.writePosition(y_angle)
    end
    
    tic
    while toc < cornerDelayTime
    frame = snapshot(cam);
    end
    
    [isFound, centroid, frame] = findBiggestObject(frame, sensitivity, numOfPixels);
    
    if isFound
        angleMap(round(x_angle*180) ,round(y_angle*180), :) = centroid(:);
        
        
        for i = 1:1:10
        for j = 1:1:10
            angleMap(round(x_angle*180)+i ,round(y_angle*180)+j, :) =...
                    angleMap(round(x_angle*180) ,round(y_angle*180), :);
        end
        end
        
        
    if (centroid(1) >= frameSize(2) - pixelsPrecision); X_ok = 1; end
    if (centroid(2) <= pixelsPrecision); Y_ok = 1; end
    end
    
	videoPlayer.step(frame);
end
disp('up right corner has found!!!');

%%
% find Down Right Corner
disp('searching for the down right corner. stege 4/5')
X_servo.writePosition(0.5)
X_servo.writePosition(0.5)
[X_ok, Y_ok, x_angle, y_angle, centroid] = parametersReset(mid_point, frameSize);
while (~Y_ok || ~X_ok)
    if centroid(1) <= frameSize(2) - pixelsPrecision
        x_angle = x_angle - delta; 
        if (x_angle > 1 || x_angle < 0); x_angle = mid_point(1); end
        X_servo.writePosition(x_angle)
    end
    if centroid(2) <= frameSize(1) - pixelsPrecision
        y_angle = y_angle + delta;
        if (y_angle > 1 || y_angle < 0); y_angle = mid_point(2); end
        Y_servo.writePosition(y_angle)
    end

    tic
    while toc < cornerDelayTime
    frame = snapshot(cam);
    end
    
    [isFound, centroid, frame] = findBiggestObject(frame, sensitivity, numOfPixels);
    
    if isFound
        angleMap(round(x_angle*180) ,round(y_angle*180), :) = centroid(:);
        
        
        for i = 1:1:10
        for j = 1:1:10
            angleMap(round(x_angle*180)+i ,round(y_angle*180)+j, :) =...
                    angleMap(round(x_angle*180) ,round(y_angle*180), :);
        end
        end
        
        
    if (centroid(1) >= frameSize(2) - pixelsPrecision); X_ok = 1; end
    if (centroid(2) >= frameSize(1) - pixelsPrecision); Y_ok = 1; end
    end
    
	videoPlayer.step(frame);
end
disp('down right corner has found!!!');

%%
% find Down Left Corner
disp('searching for the down left corner. stege 5/5');
X_servo.writePosition(0.5)
X_servo.writePosition(0.5)
[X_ok, Y_ok, x_angle, y_angle, centroid] = parametersReset(mid_point, frameSize);
while (~Y_ok || ~X_ok)
    if centroid(1) >= pixelsPrecision
        x_angle = x_angle + delta;
        if (x_angle > 1 || x_angle < 0); x_angle = mid_point(1); end
        X_servo.writePosition(x_angle)
    end
    if centroid(2) <= frameSize(1) - pixelsPrecision
        y_angle = y_angle + delta;
        if (y_angle > 1 || y_angle < 0); y_angle = mid_point(2); end
        Y_servo.writePosition(y_angle)
    end
    
    tic
    while toc < cornerDelayTime
    frame = snapshot(cam);
    end
    
    [isFound, centroid, frame] = findBiggestObject(frame, sensitivity, numOfPixels);
    
    if isFound
        angleMap(round(x_angle*180) ,round(y_angle*180), :) = centroid(:);
        
        
        for i = 1:1:10
        for j = 1:1:10
            angleMap(round(x_angle*180)+i ,round(y_angle*180)+j, :) =...
                    angleMap(round(x_angle*180) ,round(y_angle*180), :);
        end
        end
        
        
    if (centroid(1) <= pixelsPrecision); X_ok = 1; end
    if (centroid(2) >= frameSize(1) - pixelsPrecision); Y_ok = 1; end
    end
    
	videoPlayer.step(frame);
end
disp('down left corner has found!!!');

%%
% trnsfor to centroid matrix
angleMap2 = round(angleMap);
[~, ~, cor_x] = find(angleMap2(:,:,1));
[y_angle, x_angle, cor_y] = find(angleMap2(:,:,2));

convertion_table = [cor_y, cor_x, x_angle, y_angle];

disp([num2str(size(convertion_table,1)),' points was funde!'])

%{
numOfDetectedAngle = length(x_angle);
frame_col = zeros(1,numOfDetectedAngle);
frame_row = frame_col;

for i = 1:numOfDetectedAngle
    frame_col(i) = angleMap2(x_angle(i),y_angle(i),1);
    frame_row(i) = angleMap2(x_angle(i),y_angle(i),2);
end

convertion_table = [frame_row', frame_col', x_angle, y_angle];
%}

% fill missing valou
A = NaN( frameSize(1), frameSize(2), 2);
 
for i = 1:size(convertion_table,1)
    A(convertion_table(i,1), convertion_table(i,2), [1 2]) = convertion_table(i, [3 4]);
end

centroid2Angle = fillmissing(A,'linear', 1);
centroid2Angle = fillmissing(centroid2Angle,'linear', 2);

centroid2Angle(centroid2Angle<50) = 50;
centroid2Angle(centroid2Angle>150) = 150;

figure
mesh(1:size(centroid2Angle,2), 1:size(centroid2Angle,1) ,centroid2Angle(:, :, 1))
title('x angle')
xlabel('Y'); ylabel('X'); zlabel('val')
grid on

figure
mesh(1:size(centroid2Angle,2), 1:size(centroid2Angle,1) ,centroid2Angle(:, :, 2))
title('y angle')
xlabel('Y'); ylabel('X'); zlabel('val')

centroid2Angle(:,:,1) = medfilt2(centroid2Angle(:,:,1), [7 7]);
centroid2Angle(:,:,2) = medfilt2(centroid2Angle(:,:,2), [7 7]);

%linear
%nearest
figure
mesh(1:size(centroid2Angle,2), 1:size(centroid2Angle,1) ,centroid2Angle(:, :, 1))
title('x angle')
xlabel('Y'); ylabel('X'); zlabel('val')
grid on

figure
mesh(1:size(centroid2Angle,2), 1:size(centroid2Angle,1) ,centroid2Angle(:, :, 2))
title('y angle')
xlabel('Y'); ylabel('X'); zlabel('val')
%}