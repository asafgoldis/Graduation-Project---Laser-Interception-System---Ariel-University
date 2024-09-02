function Position = centroid2Position(centroid, angleMap, delta, startAngle_X, startAngle_Y)

[xAngle, yAngle] = find( (angleMap(:,:,1)<=centroid(1)+delta) &...
                         (angleMap(:,:,1)>=centroid(1)-delta) &...
                         (angleMap(:,:,2)<=centroid(2)+delta) &...
                         (angleMap(:,:,2)>=centroid(2)-delta) );
Position = [(startAngle_X+xAngle)/180, (startAngle_Y+yAngle)/180];

%%




%{
function Position = centroid2Position(xAngle, yAngle, mid_angle)

Position(1) = (mid_angle(1) - xAngle)/180;
Position(2) = (mid_angle(2) + yAngle)/180;




x = 160;
y = 120;

frameSize = [240 320];
x0 = frameSize(2)/2;
y0 = frameSize(1)/2;

wallHeight = 3;
wallWidth = 5;
wallDistance = 1;

xAngle = rad2deg(atan( (wallWidth*(x-x0))/(2*wallDistance*frameSize(2)) ));

yAngle = rad2deg(atan( (wallHeight*(y-y0))/(2*wallDistance*frameSize(1)) ));
%}

