%% find biggest object
function [isFound , centroid, frame] =...
    findBiggestObject(frame, sensitivity, numOfPixels)
                        
    centroid = [0.0 0.0];

    subtract_im = imsubtract(frame(:,:,1), rgb2gray(frame));
    bin_im = imbinarize(subtract_im, sensitivity);
    clean_bin_im = bwareaopen(bin_im, numOfPixels);
    object = regionprops(clean_bin_im, 'BoundingBox', 'Centroid');
    %'MajorAxisLength','MinorAxisLength'); 

%{
    if (x==95 & y==85)
    figure
    subplot(2,2,1); imshow(frame); title('snapshot(cam)-1')
    subplot(2,2,2); imshow(subtract_im); title('subtract im red-2')

    subplot(2,2,3); imshow(bin_im); title('bin im-3')
    subplot(2,2,4); imshow(clean_bin_im); title('clean bin im-4')
    impixelinfo;
    end
%}
    
    % find the biggest object
    bbox = zeros(length(object),4);
    for object_num = 1:length(object)
        bbox(object_num,:) = object(object_num).BoundingBox;
    end
    
    if ~isempty(bbox)
        objectSize = bbox(:,3).*bbox(:,4);
        biggestObjectNum = objectSize()==max(objectSize);
        %biggestObjectNum = find(objectSize()==max(objectSize));
        centroid = object(biggestObjectNum).Centroid;
        %diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
        %radii = diameters/2;
        
        %x_y = [x y]
        
        % Display a cross in the middel of the box. 
        frame = insertMarker(frame, centroid, '+', 'Color', 'red');
        
        centroid_text = (['X: ', num2str(round(centroid(1))),...
                      '    Y: ', num2str(round(centroid(2)))]);

        frame = insertText(frame, [centroid(1)+15 centroid(2)], centroid_text,'TextColor', 'black', 'FontSize', 12, 'BoxColor', 'red', 'BoxOpacity', 1);
    end

isFound = ~isempty(bbox);