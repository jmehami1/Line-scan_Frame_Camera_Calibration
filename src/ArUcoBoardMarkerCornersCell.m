function markerCornerCell = ArUcoBoardMarkerCornersCell(type, xNumMarker, yNumMarker, arucoLen, sepLen)
% Populate a cell array of marker corners for each marker ID present in a
% specific ArUco board. Typically, the board will have a full grid of
% markers, but it is possible to have a custom grid of marker IDs where
% specific locations on the board are always occluded.
% INPUTS:
%       type - type of ArUco board required.
%               0 : full board (default)
%               1 : wool rig platform board
%       xNumMarker - number of marker columns on board
%       yNumMarker - number of marker rows on board
%       arucoLen - side length of ArUco markers in metres
%       sepLen - distance between markers in metres
% OUTPUT:
%       markerCornerCell - cell array of 2D corner coordinates of each marker
%           in the coordinate frame of the board (world coordinates). All
%           corners have a Z-component of 0. This allows for custom boards
%           that don't require a full grid of markers.
%           [yNumMarker*xNumMarker x 1]
% Author: Jasprabhjit Mehami, 13446277

markerCornerCell = cell(yNumMarker*xNumMarker, 1);
ind = 0;

switch type
    case 0 %full board
        for i = 1:yNumMarker
            for j = 1:xNumMarker
                x_tl = (j-1)*arucoLen + (j - 1)*sepLen;
                y_tl = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_tr = j*arucoLen + (j - 1)*sepLen;
                y_tr = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_br = j*arucoLen + (j - 1)*sepLen;
                y_br = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_bl = (j-1)*arucoLen + (j - 1)*sepLen;
                y_bl = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                
                markerCorner = [
                    x_tl, y_tl;
                    x_tr, y_tr;
                    x_br, y_br;
                    x_bl, y_bl;
                    ];
                
                ind = ind + 1;
                markerCornerCell(ind) = {markerCorner};
            end
        end
        
    case 1 % platform board
        for i = 1:yNumMarker
            if any(i == 2:7)
                for j = [1, xNumMarker]
                    x_tl = (j-1)*arucoLen + (j - 1)*sepLen;
                    y_tl = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_tr = j*arucoLen + (j - 1)*sepLen;
                    y_tr = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_br = j*arucoLen + (j - 1)*sepLen;
                    y_br = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_bl = (j-1)*arucoLen + (j - 1)*sepLen;
                    y_bl = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    markerCorner = [
                        x_tl, y_tl;
                        x_tr, y_tr;
                        x_br, y_br;
                        x_bl, y_bl;
                        ];
                    
                    ind = ind + 1;
                    markerCornerCell(ind) = {markerCorner};
                end
            else
                for j = 1:xNumMarker
                    x_tl = (j-1)*arucoLen + (j - 1)*sepLen;
                    y_tl = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_tr = j*arucoLen + (j - 1)*sepLen;
                    y_tr = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_br = j*arucoLen + (j - 1)*sepLen;
                    y_br = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    x_bl = (j-1)*arucoLen + (j - 1)*sepLen;
                    y_bl = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                    
                    markerCorner = [
                        x_tl, y_tl;
                        x_tr, y_tr;
                        x_br, y_br;
                        x_bl, y_bl;
                        ];
                    
                    ind = ind + 1;
                    markerCornerCell(ind) = {markerCorner};
                end
            end
        end
        
    otherwise %default full board
        for i = 1:yNumMarker
            for j = 1:xNumMarker
                x_tl = (j-1)*arucoLen + (j - 1)*sepLen;
                y_tl = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_tr = j*arucoLen + (j - 1)*sepLen;
                y_tr = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_br = j*arucoLen + (j - 1)*sepLen;
                y_br = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                
                x_bl = (j-1)*arucoLen + (j - 1)*sepLen;
                y_bl = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
                
                markerCorner = [
                    x_tl, y_tl;
                    x_tr, y_tr;
                    x_br, y_br;
                    x_bl, y_bl;
                    ];
                
                ind = ind + 1;
                markerCornerCell(ind) = {markerCorner};
            end
        end   
end

markerCornerCell =  markerCornerCell(1:ind);
end
