%% colorDetection %%
clc; clear; close all;
clearvars all

%% SetUp Paramerers
numOfFrame = 1500;
numOfPixels = 300;  % 300
sensitivity = 0.2;  % 0.2

%% find Mid Position
%                  x    y
cam_view_angle = [113 70];
% cam_view_angle = [31.9 24.19];
guess_Position = [0.5 0.4];


cam_view_angle = deg2rad(cam_view_angle);
% xx = 2*tan(cam_view_angle(1)/2);
% yy = 2*tan(cam_view_angle(2)/2);

%%
% start serial connection
% Arduino = arduino('COM5', 'uno');
% hdlsetuptoolpath('ToolName','Altera Quartus II','ToolPath','C:\intelFPGA_lite\17.1\quartus\bin64\quartus.exe');

% hdlsetuptoolpath('ToolName','Altera Quartus II','ToolPath','C:\intelFPGA_lite\17.1\quartus\bin64\quartus.exe');

% Serial object
delete(instrfind);
% s = serial('COM5', 'BaudRate', 9600, 'DataBits',8);
s = serial('COM5', 'BaudRate', 115200, 'DataBits',8);
% Close any open objects so they don't interfere
% fclose(instrfind);
% get(s)

% instrfind
% seriallist
fopen(s);
% delete(instrfind);
% fclose('all');



%X_servo = Arduino.servo(9, 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3)
%Y_servo = Arduino.servo(10, 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3)
% X_servo = Arduino.servo('D9');
% Y_servo = Arduino.servo('D10');

% Create the webcam object.
%cam = webcam('Integrated Webcam', 'Resolution', '640x480');   % '1280x720' % '640x480'
%cam = webcam('Logitech QuickCam Express/Go', 'Resolution', '640x480');
%cam = webcam('USB_Camera', 'Resolution', '1280x720');
cam = webcam('USB_Camera', 'Resolution', '640x480');

% Capture one frame to get its size.
frame = snapshot(cam);
frameSize = size(frame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
%videoFWriter = vision.VideoFileWriter('recorde.avi', 'FrameRate', 20);

% mid_Position = findMidPosition(X_servo, Y_servo, cam, videoPlayer, guess_Position);
mid_Position = [0.46 0.45]; % 0.46 0.45

xx = mid_Position(1) + 0.5*cam_view_angle(1)/pi;
yy = mid_Position(2) - 0.5*cam_view_angle(2)/pi;

xxx = cam_view_angle(1)/(frameSize(2)*pi);
yyy = cam_view_angle(2)/(frameSize(1)*pi);

%%
frameCount = 0;
runLoop = 1;

% Set a loop that stop after 100 frames of aquisition
while(frameCount<numOfFrame && runLoop) 
tic
    frameCount = frameCount + 1;
    
    % Get the snapshot of the current frame
    frame = snapshot(cam);
    
    % subtract the red component from the grayscale
    % image to extract the red components in the image.
    subtract_im = imsubtract(frame(:,:,1), rgb2gray(frame));  %1-red, 2-, 3-bluo
    
    % Convert the resulting grayscale image into a binary image.
    bin_im = imbinarize(subtract_im, sensitivity);
    
    % Remove all those pixels less than 300px
    clean_bin_im = bwareaopen(bin_im, numOfPixels);
    
    % get a set of properties for each labeled region.
    object = regionprops(clean_bin_im, 'BoundingBox', 'Centroid', 'Area');
    
    %% Display the biggest object
    if ~isempty(object)
        biggestObjectNum = [object.Area]==max([object.Area]);
        
        boundingBox = object(biggestObjectNum).BoundingBox;
        centroid = object(biggestObjectNum).Centroid;
        
        % Display a bounding box around the object being tracked. 
        frame = insertShape(frame, 'Rectangle', boundingBox, 'LineWidth', 5, 'color', 'red');
        
        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');
        
        % Display center mass centroid. 
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);
        
        frame = insertText(frame, [centroid(1)+15 centroid(2)],...
            centroid_text,'TextColor', 'black', 'FontSize', 12,...
            'BoxColor', 'red', 'BoxOpacity', 1);
        
%         xAngle = atan(xx*(centroid(1)/frameSize(2)-0.5));
%         yAngle = atan(yy*(centroid(2)/frameSize(1)-0.5));
  
%         X_servo.writePosition(mid_Position(1) - xAngle/pi)
%         Y_servo.writePosition(mid_Position(2) + yAngle/pi)
        
        
        
        x = uint8((xx - xxx*centroid(1))*128);
        y = uint8((yy + yyy*centroid(2))*128);
        sendPos(s, x, '1');
        sendPos(s, y, '0');
        

        
        
%         X_servo.writePosition(xx - xxx*centroid(1))
%         Y_servo.writePosition(yy + yyy*centroid(2))
    else
        sendPos(s, 0, '0');

    end
    
    videoPlayer.step(frame);
    %videoFWriter.step(frame);
runLoop = isOpen(videoPlayer);
% toc
end    % loop end here.

% Clean up.
clear cam;
% clear Arduino;
release(videoPlayer);
%clear videoPlayer;
%clear X_servo;
%clear Y_servo;
%clear object;
%release(videoFWriter);


function sendPos(s, pos, xy)
    OK = 240;
    NOT_OK = 255;
    
    pos = bin2dec([dec2bin(pos,7) xy]);
        loop = true;
        while (loop)
            flushoutput(s)
            fwrite(s, pos)
            %pause(0.1)
            flushinput(s)
            readData = fread(s,1,'uint8');
            
            if readData == OK; loop = false;
            elseif readData == NOT_OK; loop = true;
            end
        end

    % disp('OK')
end
