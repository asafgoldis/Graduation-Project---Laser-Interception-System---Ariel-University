%clc
%clear all
%close all
clearvars all

arduino_nano = serial('COM5','BaudRate',115200);
fopen(arduino_nano);
readData = arduino_nano.fscanf() %reads "Ready" 


x = 85  % 640  90
y = 95  % 480  95


%x = 115 - x*50/640
%y = 130 + y*40/480

fwrite(arduino_nano, x)
fwrite(arduino_nano, y)

%angleX = fscanf(arduino, '%u')
%angleY = fscanf(arduino, '%u')

%{
while (angleX ~= x || angleY~= y)
    fwrite(arduino, x)
    fwrite(arduino, y)
    
    angleX = fscanf(arduino, '%u')
    angleY = fscanf(arduino, '%u')
end
%}

%{
for j=0:20:640
    
x = j*180/640
%x = floor(x*180/640)
fwrite(arduino, x)

angle = fscanf(arduino, '%u');
%while ( angle ~= x )
%    a=a+1
%    angle = str2double(fread(arduino))
%end
%a=0;
angle
end
%}

%fwrite(arduino, 0)
%angle = fscanf(arduino, '%u');

%X = fscanf(arduino)
%{
if x > 255
x_L = str2double(fscanf(arduino));
x_H = str2double(fscanf(arduino));
X = x_H*256 + x_L
else
X = str2double(fscanf(arduino))
end
%}

%fwrite(arduino, 90)
%fwrite(arduino, 90)

%angleX = fscanf(arduino, '%u')
%angleY = fscanf(arduino, '%u')

fclose(arduino_nano);
delete(arduino_nano);
 