function DisplayPattern(fig, pattern, transPts, ext, is2D)
%Plot the calibration pattern onto a 3D plot using the provided
%dimensions. Draw the view-line by plotting the intersection points that
%was seen by the linescan camera.
%INPUTS:
%       fig - input figure used to display all objects
%       pattern - object containing parameters of the calibration pattern
%       transPts - transformed feature points of the view-line in the
%           coordinate frame of the frame camera
%       ext - Extrinsic transform that when applied will transform pattern
%           points to the frame camera coordinate frame
%Author: Jasprabhjit Mehami, 13446277

figure(fig);

xNumCheck = pattern.NumCols;
yNumCheck = pattern.NumRows;
arucoSideLen = pattern.ArucoSideLength;
sepLen = pattern.SeparationLength;
numLines = pattern.numLines;
xOff = pattern.Xoffset;
yOff = pattern.Yoffset;
triWidth = pattern.XTriWidth;
triHeight = pattern.YTriHeight;

%plot on 2D axis
if is2D
    %plot the position of the points on the pattern
    plot(transPts(:,1), transPts(:,2), '-x', 'LineWidth', 1.5, 'MarkerFaceColor', [0,1,0]); hold on;
    
    %axis at origin
    trplot(ext, 'frame', 'Pat', 'rviz', 'length', 0.1);
    
    %edge points on pattern
    bl = [0, 0, 1];
    br = [xNumCheck*arucoSideLen + (xNumCheck-1)*sepLen, 0, 1];
    tl = [0, yNumCheck*arucoSideLen + (yNumCheck-1)*sepLen, 1];
    tr = [xNumCheck*arucoSideLen + (xNumCheck-1)*sepLen, yNumCheck*arucoSideLen + (yNumCheck-1)*sepLen, 1];
    
    %transform edge points
    coor = ([bl ; br; tr; tl ]') ;
    
    %plot coloured surface
    fill(coor(1,:),coor(2,:),'r', 'FaceAlpha', 0.1)
    
    
    
    %fill array with triangle points from the pattern
    triPts = zeros(4*numLines/2,3);
    
    for i = 1:numLines/2
        triPts((i-1)*4+1:i*4, 1:2) =  ...
            [
            xOff + (i-1)*triWidth, yOff;
            xOff + (i-1)*triWidth, yOff + triHeight;
            xOff + i*triWidth, yOff;
            xOff + (i-1)*triWidth, yOff;
            ];
    end
        
    %plot triangles
    plot(triPts(:,1), triPts(:,2), '-k', 'LineWidth', 1.5); hold on;
    
    

else
    
    %plot the position of the points on the pattern
    plot3(transPts(:,1), transPts(:,2), transPts(:,3), '-x', 'LineWidth', 1.5, 'MarkerFaceColor', [0,1,0]); hold on;
    
    %axis at origin
    trplot(ext, 'frame', 'Pat', 'rviz', 'length', 0.1); hold on;
    
    %edge points on pattern
    bl = [0, 0, 0, 1];
    br = [xNumCheck*arucoSideLen + (xNumCheck-1)*sepLen, 0, 0,  1];
    tl = [0, yNumCheck*arucoSideLen + (yNumCheck-1)*sepLen, 0,  1];
    tr = [xNumCheck*arucoSideLen + (xNumCheck-1)*sepLen, yNumCheck*arucoSideLen + (yNumCheck-1)*sepLen, 0,  1];
    
    %transform edge points
    coor = ext*([bl ; br; tr; tl ]') ;
    
    %plot coloured surface
    fill3(coor(1,:),coor(2,:),coor(3,:),'r', 'FaceAlpha', 0.1)
    
    
    
    %fill array with triangle points from the pattern
    triPts = zeros(4*numLines/2,3);
    
    for i = 1:numLines/2
        triPts((i-1)*4+1:i*4, 1:2) =  ...
            [
            xOff + (i-1)*triWidth, yOff;
            xOff + (i-1)*triWidth, yOff + triHeight;
            xOff + i*triWidth, yOff;
            xOff + (i-1)*triWidth, yOff;
            ];
    end
    
    transPts = (ext*[triPts'; ones(1, length(triPts(:,1)))])';
    
    %plot triangles
    plot3(transPts(:,1), transPts(:,2), transPts(:,3), '-k', 'LineWidth', 1.5); hold on;
    
end

    axis equal; grid on;
    drawnow();
end