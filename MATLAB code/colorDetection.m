%% colorDetection %%
clc; clear; close all;
clearvars all

%% SetUp Paramerers
sensitivity = 0.3;  % Threshold for Binary Img. - 0.2
numOfPixels = 35;  % Pixel Threshold - 300; For 5m need less then 50

%% Serial object
% Close any open objects so they don't interfere
delete(instrfind);

% start serial connection 256000
SerialConn = serial('COM4', 'BaudRate', 256000 , 'DataBits', 8);
fopen(SerialConn);

%% Create the webcam object.
% cam = webcam('USB_Camera', 'Resolution', '1280x720');
% cam = webcam('USB_Camera', 'Resolution', '800x600');
cam = webcam('USB_Camera', 'Resolution', '640x480');

% Capture one frame to get its size.
frame = snapshot(cam);
frameSize = size(frame);
midFrame = flip(frameSize(1:2))/2;

%% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
videoPlayer.step(frame);
% videoFWriter = vision.VideoFileWriter('recorde.avi', 'FrameRate', 20);

%% preset
Position = [0.5 0.5];
for i=0:5
    sendPos(SerialConn, 0.5, '01');
end

%%
Count = 0;
runLoop = 1;
frameCount = 0;
% Set a loop that stop after 3500 frames of aquisition
while(runLoop)
%tic
    frameCount = frameCount + 1;
    % Get the snapshot of the current frame
    frame = snapshot(cam);
    
    % subtract the red component from the grayscale image to extract the red components in the image.
    subtract_im = imsubtract(frame(:,:,1), rgb2gray(frame));  %1-Red, 2-Green, 3-Blue
    
    % Convert the resulting grayscale image into a binary image.
    bin_im = imbinarize(subtract_im, sensitivity);
    
    % Remove all those pixels less than 35px
    clean_bin_im = bwareaopen(bin_im, numOfPixels, 4);
    
%{
if (mod(frameCount,200) == 0)
    subtract_im_green = imsubtract(frame(:,:,2), rgb2gray(frame));  %1-Red, 2-Green, 3-Blue
    subtract_im_blue = imsubtract(frame(:,:,3), rgb2gray(frame));  %1-Red, 2-Green, 3-Blue    

    
    figure
    subplot(3,3,1); imshow(frame); title('Orginal Frame')
    subplot(3,3,3); imshow(rgb2gray(frame)); title('Gray Image')

    subplot(3,3,4); imshow(frame(:,:,1)); title('Red Component')
    subplot(3,3,5); imshow(frame(:,:,2)); title('Green Component')
    subplot(3,3,6); imshow(frame(:,:,3)); title('Blue Component')

    subplot(3,3,7); imshow(subtract_im); title('Extract Red')
    subplot(3,3,8); imshow(subtract_im_green); title('Extract Green')
    subplot(3,3,9); imshow(subtract_im_blue); title('Extract Blue')
    impixelinfo;


    figure
    subplot(2,2,1); imshow(frame); title('Step One: Orginal Frame')
    subplot(2,2,2); imshow(subtract_im); title('Step Two: Extract Red Component (subtract)')

    subplot(2,2,3); imshow(bin_im); title('Step Three: Binary Image')
    subplot(2,2,4); imshow(clean_bin_im); title('Step Four : Clean Image')
    impixelinfo;
end
%}
    
    % get a set of properties for each labeled region.
    object = regionprops(clean_bin_im, 'BoundingBox', 'Centroid', 'Area');
    
    %% Display the biggest object
    if ~isempty(object)
        Count = 0;
        biggestObjectNum = [object.Area]==max([object.Area]);

        centroid = object(biggestObjectNum).Centroid;
        boundingBox = object(biggestObjectNum).BoundingBox;
        
        % Display a bounding box around the object being tracked. 
        frame = insertShape(frame, 'Rectangle', boundingBox, 'LineWidth', 5, 'color', 'red');
        
        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');
        
        % Display center object centroid. 
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);
        
        frame = insertText(frame, [centroid(1)+15 centroid(2)],...
            centroid_text,'TextColor', 'black', 'FontSize', 12,...
            'BoxColor', 'red', 'BoxOpacity', 1);
        
        for i=1:2
            if (abs(centroid(i)-midFrame(i)) > 15)  % 15 pixels
                if centroid(i) > midFrame(i)
                    centroid(i) = centroid(i) + 8; % 8 pixels
                    if centroid(i) > frameSize(i)
                        centroid(i) = frameSize(i);
                    end
                else
                    centroid(i) = centroid(i) - 8; % 8 pixels
                    if centroid(i) < 0
                        centroid(i) = 0;
                    end
                end
            end
        end
        
        step = 0.06*(centroid./midFrame - 1);
        step(abs(centroid-midFrame) < 7) = 0;   % midFrame./[36 27]=8.88
        
        Position = Position + step;
        
        % Limit Motor Angle
        Position(Position>0.75) = 0.75;  % 0.5 + 45/180 = 0.75 % +45 deg
        Position(Position<0.25) = 0.25;  % 0.5 - 45/180 = 0.25 % -45 deg
        %Position(Position>1) = 1;
        %Position(Position<0) = 0.0001;
        
        sendPos(SerialConn, Position(1), '11');
        sendPos(SerialConn, Position(2), '10');
    else
        sendPos(SerialConn, 0, '00');
        Count = Count + 1;
        if (115<Count && Count<120)     % 0.035 sec for loop, 0.035*115=4.025 sec
            Position = [0.5 0.5];
            sendPos(SerialConn, 0.5, '01');
        end
    end
    
    videoPlayer.step(frame);        % update display
%     videoFWriter.step(frame);
    
runLoop = isOpen(videoPlayer);
% toc
end    % loop end here.

% Reset Position
for i=0:10
    sendPos(SerialConn, 0.5, '01');
end
%% Clean up.
clear cam;
release(videoPlayer);
% release(videoFWriter);

function sendPos(port, pos, xy)
    pos = bin2dec([dec2bin(uint8(pos*63),6) xy]);
    flushoutput(port)
    fwrite(port, pos)
end
