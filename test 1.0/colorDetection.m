%% colorDetection %%

clc
clear all
close all
clearvars all

%% SetUp Paramerers
numOfFrame = 1500;
numOfPixels = 300;  % 300
sensitivity = 0.2;  % 0.2

%% laserCalibration
startAngle_X = 70;  % 30
stopAngle_X = 106;  % 106
startAngle_Y = 70;  % 50
stopAngle_Y = 97;   % 97
%precision = 1;

cam_view_angle_x = 32;
cam_view_angle_y = 25;
mid_angle = findMidAngle();

cam_view_angle_x = deg2rad(cam_view_angle_x);
cam_view_angle_y = deg2rad(cam_view_angle_y);
mid_angle = deg2rad(mid_angle);

xx = 2*tan(cam_view_angle_x/2);
yy = 2*tan(cam_view_angle_y/2);

%% centroid2Position
delta = 5;

%%

% start serial connection
Arduino = arduino('COM5', 'uno');
%X_servo = Arduino.servo(9, 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3)
%Y_servo = Arduino.servo(10, 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3)
X_servo = Arduino.servo('D9');
Y_servo = Arduino.servo('D10');

% Create the webcam object.
%cam = webcam('Integrated Webcam', 'Resolution', '640x480');   % '1280x720' % '640x480'
cam = webcam('Logitech QuickCam Express/Go', 'Resolution', '320x240');
%cam = webcam('Logitech QuickCam Express/Go', 'Resolution', '640x480');

% Capture one frame to get its size.
frame = snapshot(cam);
frameSize = size(frame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
%videoFWriter = vision.VideoFileWriter('recorde.avi', 'FrameRate', 20);

%%
%{
mid_centroid = [frameSize(2)/2 frameSize(1)/2];
mid_angle = [82 79];

wallHeight = 0.7;
wallWidth = 0.95;
wallDistance = 1.7;
%}

%angleMap = laserCalibration(...
%    X_servo, Y_servo, cam, videoPlayer,...
%    startAngle_X, stopAngle_X, startAngle_Y,stopAngle_Y);
%{
figure
mesh(1:size(angleMap,2), 1:size(angleMap,1) ,angleMap(:, :, 1))
figure
mesh(1:size(angleMap,2), 1:size(angleMap,1) ,angleMap(:, :, 2))
disp('max:')
max(max(max(angleMap)))
disp('min:')
min(min(min(angleMap)))
%}

disp('laser Calibration complite!!!')

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
    
    % Use a median filter to filter out noise
    %medfilter = medfilt2(subtract_im, [3 3]);
    
    % Convert the resulting grayscale image into a binary image.
    bin_im = imbinarize(subtract_im, sensitivity);      %0.18    0.3

    % Remove all those pixels less than 300px
    clean_bin_im = bwareaopen(bin_im, numOfPixels);      % 300   500

%{
    
subtract_im_green = imsubtract(frame(:,:,2), rgb2gray(frame));  %1-red, 2-, 3-blue
subtract_im_blou = imsubtract(frame(:,:,3), rgb2gray(frame));  %1-red, 2-, 3-blue    
    
if (frameCount<2)
figure
subplot(3,3,1); imshow(frame); title('snapshot(cam)')
subplot(3,3,3); imshow(rgb2gray(frame)); title('gray only')

subplot(3,3,4); imshow(frame(:,:,1)); title('red only')
subplot(3,3,5); imshow(frame(:,:,2)); title('green only')
subplot(3,3,6); imshow(frame(:,:,3)); title('blue only')

subplot(3,3,7); imshow(subtract_im); title('subtract red')
subplot(3,3,8); imshow(subtract_im_green); title('subtract green')
subplot(3,3,9); imshow(subtract_im_blou); title('subtract blue')
impixelinfo;


figure
subplot(2,2,1); imshow(frame); title('snapshot(cam)-1')
subplot(2,2,2); imshow(subtract_im); title('subtract im red-2')

subplot(2,2,3); imshow(bin_im); title('bin im-3')
subplot(2,2,4); imshow(clean_bin_im); title('clean bin im-4')
impixelinfo;

end
%}
    
    % get a set of properties for each labeled region.
    object = regionprops(clean_bin_im, 'BoundingBox', 'Centroid');
    
    %% Display the biggest object
    
    % find the biggest object
    bbox = zeros(length(object),4);
    for object_num = 1:length(object)
        bbox(object_num,:) = object(object_num).BoundingBox;
    end
    
    if ~isempty(bbox)
        objectSize = bbox(:,3).*bbox(:,4);
        biggestObjectNum = find(objectSize()==max(objectSize));

        boundingBox = object(biggestObjectNum).BoundingBox;
        centroid = object(biggestObjectNum).Centroid;

        % Display a bounding box around the object being tracked. 
        frame = insertShape(frame, 'Rectangle', boundingBox, 'LineWidth', 5, 'color', 'red');

        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');

        % Display center mass centroid. 
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);

        frame = insertText(frame, [centroid(1)+15 centroid(2)], centroid_text,'TextColor', 'black', 'FontSize', 12, 'BoxColor', 'red', 'BoxOpacity', 1);
        
        % angle 
%xAngle = rad2deg(atan( (wallWidth*(centroid(1)-mid_centroid(1)))/...
%                       (2*wallDistance*frameSize(2)) ));
%yAngle = rad2deg(atan( (wallHeight*(centroid(2)-mid_centroid(2)))/...
%                       (2*wallDistance*frameSize(1)) ));
        
%        Position = centroid2Position(xAngle, yAngle, mid_angle);

        %Position = centroid2Position(centroid, angleMap, delta, startAngle_X, startAngle_Y);
        
        %Position = [angleMap(round(centroid(2)), round(centroid(1)) ,1) ...
        %            angleMap(round(centroid(2)), round(centroid(1)) ,2)];
        %Position = Position/180;

centroid

xAngle = mid_angle(1) + atan(xx*(centroid(1)/frameSize(2)-0.5));
yAngle = mid_angle(2) + atan(yy*(centroid(2)/frameSize(1)-0.5));

xAngle = pi - xAngle;

xyAngle = [xAngle yAngle]
Position = xyAngle/pi

        if ~isempty(Position)
            X_servo.writePosition(Position(1))
            Y_servo.writePosition(Position(2))
        end

    end
    
    
    %% Display all object   
    %{
    %This is a loop to bound the red objects in a rectangular box.
    for object_num = 1:length(object)
        boundingBox = object(object_num).BoundingBox;
        centroid = object(object_num).Centroid;
        
        % Display a bounding box around the object being tracked. 
        frame = insertShape(frame, 'Rectangle', boundingBox, 'LineWidth', 5, 'color', 'red');

        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');

        % Display center mass centroid. 
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);

        frame = insertText(frame, [centroid(1)+15 centroid(2)], centroid_text,'TextColor', 'black', 'FontSize', 12, 'BoxColor', 'red', 'BoxOpacity', 1);
    end
    %%
    %}
    
    videoPlayer.step(frame);
    %videoFWriter.step(frame);
    runLoop = isOpen(videoPlayer);
toc    
end    % loop end here.

% Clean up.
clear cam;
clear Arduino;
release(videoPlayer);
%clear videoPlayer;
%clear X_servo;
%clear Y_servo;
%clear object;
%release(videoFWriter);

%{
phi_x=100;
phi_y=100;
d = 3;
n_max = 320;
m_max = 240;
dx = 4*d*tan(phi_x/2)/n_max;
dy = 4*d*tan(phi_y/2)/m_max;
centroid = [100 200];

syms n m
f_x = (dx/sqrt(d^2 + (n*dx)^2))
f_y = (dy/sqrt(d^2 + (centroid(2)*dx)^2 + (centroid(1)*dy)^2))

theta_x = symsum((dx/sqrt(d^2 + (n*dx)^2)), n, 0, centroid(2))
theta_y = symsum((dy/sqrt(d^2 + (centroid(2)*dx)^2 + (centroid(1)*dy)^2)), m, 0, centroid(1))
%}




%% arduino code no needed
%{

#include <Servo.h>
Servo myServoX;
Servo myServoY;

const int ServoXPin = 9;
const int ServoYPin = 10;
const int delayTime = 4;

int centroidX;
int centroidY;
//int angleX;
//int angleY;

void setup(){
    myServoX.attach(ServoXPin);
    myServoY.attach(ServoYPin);
    Serial.begin(115200);
    Serial.println("Ready");
}

void loop() {
    if (Serial.available() > 0) {
        centroidX = Serial.read();
        delay(delayTime);
        centroidY = Serial.read();
        //delay(delayTime);

        myServoX.write(centroidX);
        myServoY.write(centroidY);
        //delay(delayTime);

        //angleX = myServoX.read();
        //angleY = myServoY.read();

        //Serial.println(angleX); //print data
        //delay(delayTime);
        //Serial.println(angleY); //print data
        //delay(delayTime);
    }
}

%}
