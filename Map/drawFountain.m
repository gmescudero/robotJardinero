clearvars

mapSize.scale = 50;
mapSize.placingResolution = 100;
mapSize.x = round(26.5*mapSize.scale);
mapSize.y = round(13.5*mapSize.scale);

BW = zeros(mapSize.x,mapSize.y);

fountain.center = [ round(10.35 * mapSize.scale),...
                    round(6.75  * mapSize.scale)        ];
fountain.radius = round(1.0 * mapSize.scale);

% Create the image.
[columnsInImage, rowsInImage] = meshgrid(1:mapSize.x, 1:mapSize.y);
% Create the circle in the image.
circlePixels = (rowsInImage - fountain.center(2)).^2 ...
    + (columnsInImage - fountain.center(1)).^2 <= fountain.radius.^2;

colormap([0 0 0; 1 1 1]);
title('Binary image of a circle');