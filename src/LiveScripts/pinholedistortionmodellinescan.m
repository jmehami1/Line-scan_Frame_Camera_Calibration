function [u_d, v_d] = pinholedistortionmodellinescan(X, Y, Z, t1, t2, t3, R, fy, v0, K1, K2, T1)
% Pinhole and brown-conrady distortion camera model for a line-scan camera
% INPUTS:
%       X, Y, Z - 3D coordinates captured by the camera
%       t1, t2, t3 - Translation component of the extrinsics
%       R - Rotation matrix of extrinsics
%       fy - focal length of camera in v-direction 
%       v0 - optical centre in v-direction
%       K1, K2 - Radial distortion components
%       T1 - Tangential distortion component
% OUTPUT:
%       u_d, v_d - distorted u and v image coordinates

%extract rotation matrix elements
r11 = R(1,1);
r12 = R(1,2);
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);

%undisorted v component
v = v0 + fy.*(X.*r21 + Y.*r22 + Z.*r23 + t2)./(X.*r31 + Y.*r32 + Z.*r33 + t3);

%normalised v image coordinate
y = (v - v0)./fy;

%normalised v image coordinate distorted
y_d = y*(1 + K1*y^2 + K2*y^4) + 3*T1*y^2;

%disorted v component
v_d = y_d*fy + v0;

%calculate the u coordinate of the projected points uisng the pinhole
%model (no distortion occurs here)
u = fy.*(X.*r11 + Y.*r12 + Z*r13 + t1)./(X.*r31 + Y.*r32 + Z.*r33 + t3);
u_d = u;

end