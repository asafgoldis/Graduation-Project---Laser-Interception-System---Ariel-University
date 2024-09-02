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

%[a, b, c] = find(centroid2Angle>180)

%[a, b, c] = find(centroid2Angle<1)

%{
centroid2Angle(centroid2Angle(:,:,:)>180) = NaN;
centroid2Angle = fillmissing(centroid2Angle,'nearest', 2);
centroid2Angle = fillmissing(centroid2Angle,'nearest', 1);

figure
mesh(centroid2Angle(:,:,1))

if find(centroid2Angle(:,:,:)>180)
    disp('ERROR')
end
%}
%aa = find(centroid2Angle(:,:,2)>180)
%[bb_r bb_c] = centroid2Angle(:,:,2)>180;
%mesh(angleMap(:,:,1))

%%
%{
a=magic(8)
b=(a>30).*a
b(b==0)=NaN

a=magic(30);
b=(a>30).*a;
b(b==0)=NaN;
x=1:30;
y=x;
mesh(x,y,b)
%}
%%

%{
A = [0 0 0 0 0;
     0 0 4 0 0;
     0 0 0 0 0;
     0 0 0 0 0;
     0 0 3 0 0]
 
A(A==0) = NaN

B = fillmissing(A,'linear', 2)
F = fillmissing(B,'linear', 1)
%}
