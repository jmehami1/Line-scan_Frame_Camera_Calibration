function Plot3DAxis2Image(T, len, K, imgSize, dist)
% Plots a projected RGB 3D axis to a 2D image taken by a camera
% by applying an extrinsic transformation.
% INPUTS:
%       T - extrinsic transformation from world coordinates to camera
%       coordinates
%       len - length of the axis in world coordinates

%create homogenous points representing the edges of the basis vectors for
%the 3D axis AND scale according to len
axis3Dhom = [[0;0;0],len .* eye(3)];
axis3Dhom = [axis3Dhom; 1, 1, 1, 1];

%project points to image coordinates
axis3DPix = projectPoints(axis3Dhom', K, T, dist, imgSize);

%Edges of basis vectors in image coordinates
oPix = axis3DPix(1,1:2);
xPix = axis3DPix(2,1:2);
yPix = axis3DPix(3,1:2);
zPix = axis3DPix(4,1:2);

%plot the lines of the axis in RGB colour format
plot2([oPix; xPix], 'Color', 'r', 'LineWidth', 5);
plot2([oPix; yPix], 'Color', 'g', 'LineWidth', 5);
plot2([oPix; zPix], 'Color', 'b', 'LineWidth', 5);

end

