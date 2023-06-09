function [intsecPtPlane, valid] = CameraPixelRayIntersectPlane(normImgPt, planeSurfNorm, planePt)
% Determine the 3D location of the intersection of a pixel ray to a flat
% plane. Pixel is given as normalised coordinates, which remove the effects
% of the intrinsic parameters and places the points on a virtual plane that
% is one unit infront of the camera (not metres). The plane is
% parameterised by its surface normal and a single point lying on the
% plane. All points and vectors should be described in the coordinate
% frame of the camera.
% INPUTS:
%       normImgPt - normalised image coordinate (x,y,1) (1x3)
%       planeSurNorm - surface normal of plane w.r.t to the camera's
%       coordiante frame (1x3)
%       planePt - point on plane w.r.t to the camera's coordinate frame (1x3)
% OUTPUTS:
%       intsecPtPlane - intersected 3D point between ray and plane (1x3) w.r.t to
%       camera's coordinate frame
%       valid - valid intersection where only one intersection point is returned

%Author: Jasprabhjit Mehami, 13446277

%direction vector of ray starts at camera origin towards normalised image
%coordinate
rayDir = normImgPt;

%calculate direction vector of ray/line using normalised image coordinates
% rayDir = normImgPt./norm(normImgPt);

%line plane intersection. The camera origin is treated as the starting point of the
%ray

[intsecPtPlane, rc] = LinePlaneIntersection(rayDir, [0,0,0], planeSurfNorm, planePt);

if rc == 1
    valid = true;
    return;
end

%intersection was not valid. 
valid = false;

end

