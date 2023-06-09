function [patternPts, var_patternPts] = CrossRatioPatternPoints(pattern, vImagePts, varImgPts)
%Determine the points on the pattern which are seen by the line-scan
%camera using knowledge of the pattern and line-scan image. Also calculate
%the uncertainty related to these points.
%INPUTS:
%       pattern - object containing parameters of the calibration pattern
%       vImagePts - v coordinates of feature points on image line
%       varImgPts - variance error in the measured pixel coordinates
%OUTPUTS:
%       patternPts - [X,Y,0] coordinates calculated from the linescan camera
%       image points.2D array where first column is the X-coordinates,
%       second column contains the corresponding Y-coordinates, and the
%       third column has the Z-coordinates
%       err_patternPts - covariance matrix associated for each calculated
%       point on the pattern.
%Author: Jasprabhjit Mehami, 13446277

%pattern dimensions and parameters
xl = pattern.XTriWidth;
h = pattern.YTriHeight;
xOff = pattern.Xoffset;
yOff = pattern.Yoffset;
n = pattern.numLines;

%preallocate space for storing the calculated coordinates
X = zeros(1,n);
Y = zeros(1,n);
Z = zeros(1,n); %Z coordinates always at 0

%variance of each component
varX = zeros(1,n);
varY = zeros(1,n);
varZ = zeros(1,n); %variance for z coordinate is always 0% errM = err(1,1);


%We know the Y component of all intersections with odd numbered lines (vertical lines) from
%the pattern
%variance of this component is zero
for i = 1:2:n-1
    X(i) = ((i-1)/2)*xl + xOff;
end

%determine the y components of all intersections with even numbered lines
%(diagonal lines) from the pattern, EXCEPT FOR THE LAST TWO
for i = 2:2:n-4
    %feature pixels
    a = vImagePts(i-1);
    b = vImagePts(i);
    c = vImagePts(i+1);
    d = vImagePts(i+3);
    
    %calculate cross ratio
    nu = ((c-a)*(d-b))/((c-b)*(d-a));
    
    %feature pattern points
    A = ((i/2)-1)*xl + xOff;
    C = ((i/2))*xl + xOff;
    D = ((i/2)+1)*xl + xOff;
    
    X(i) = (nu*C*(A-D) + D*(C-A))/(nu*(A-D) + (C-A));
    
    jac = JacobianCrossRatio_B([a, b, c, d], [A, C, D]);
    varX(i) = Calc_Error_Y(jac, varImgPts);
end

%determine the X componet of the second-to-last intersection line
%(diagonal)
a = vImagePts(n-5);
b = vImagePts(n-3);
c = vImagePts(n-2);
d = vImagePts(n-1);

nu = ((c-a)*(d-b))/((c-b)*(d-a));

A = (n/2-3)*xl + xOff;
B = (n/2-2)*xl + xOff;
D = (n/2-1)*xl + xOff;

X(n-2) = (nu*B*(D-A) + A*(B-D))/((B-D) + nu*(D-A));

jac = JacobianCrossRatio_C([a, b, c, d], [A, B, D]);
varX(n-2) = Calc_Error_Y(jac, varImgPts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%Determine the X componet of the last intersection line (diagonal line)
a = vImagePts(n-5);
b = vImagePts(n-3);
c = vImagePts(n-1);
d = vImagePts(n);

nu = ((c-a)*(d-b))/((c-b)*(d-a));

A = (n/2-3)*xl + xOff;
B = (n/2-2)*xl + xOff;
C = (n/2-1)*xl + xOff;

X(n) = (nu*A*(C-B) + B*(A-C))/((A-C) + nu*(C-B));

jac = JacobianCrossRatio_D([a, b, c, d], [A, B, C]);
varX(n) = Calc_Error_Y(jac, varImgPts);

%Determine the Y component of all diagonal lines
for i = 2:2:n
    %define both end points of the current diagonal line in the pattern
    y2 = yOff+h;
    x2 = ((i/2) - 1)*xl + xOff;
    
    x1 = (i/2)*xl + xOff;
    y1 = yOff;
    %find the line equation between both points
    coef = polyfit([x1, x2], [y1,y2], 1);
    
    %coefficients of the linear equation
    m = coef(1);
    c = coef(2);
    
    Y(i) = m*X(i) + c;

    %rearrange to calculate X
%     X(i) = (Y(i) - c)/m;
    
    %The only uncertain variable is X. Both calculated m and c have no
    %associated error
    varY(i) = (1/(m^2))*varX(i);
    
end

%extract the currently known points on the pattern (points on diagonal lines)
xData = X(2:2:n);
yData = Y(2:2:n);

%fit the linear equation to determine the view-line
%TODO: polyfit returns error, check if calculated error in [m,c] is similar
%to error by polyfit
coef = polyfit(xData, yData, 1);
m = coef(1);
c = coef(2);

nData = length(xData); %number of data points that will be used to fit a line

%This should be equal to 4, else you have to get new jacobian equation
if nData == 4
    jac = JacobianLineLeastSquaresFit(xData, yData);
    cov = diag([varX(2:2:n),varY(2:2:n)]);
    err = jac*cov*jac'; %[ m ; c]
else
    %There should only be 4 points otherwise new jacobian function needs to
    %be determined
    %Need to determine the error in the linear least squares fit for the data
    syms 'x%d' [nData,1] 'real'
    syms 'y%d' [nData,1] 'real'
    
    W = [y, ones(nData,1)];
    coef_ = (W'*W)\(W'*x); %[m,c]
    jac = jacobian( coef_, [x;y]);
    jac = subs(jac, x, xData');
    jac = subs(jac, y, yData');
    jac = eval(jac);
    cov = diag([varX(2:2:n),varY(2:2:n)]);
    err = jac*cov*jac'; %[ m ; c]
end


%Determine the X component of the straight lines using the fitted view-line equation
for i = 1:2:n-1
    %rearrange to calculate the X
    %X(i) = (Y(i) - c)/m;
    Y(i) = m*X(i) + c;
    
    %Calculate the variance in X
    jac = JacobianLinearEqY_Value(m,c,X(i));
    varY(i) = jac*err*jac';
    
end

patternPts(:,1) = X;
patternPts(:,2) = Y;
patternPts(:,3) = Z;

var_patternPts(:,1) = varX;
var_patternPts(:,2) = varY;
var_patternPts(:,3) = varZ;

end

%Calculate the variance of Y from the variance of the image pixels
function varY = Calc_Error_Y(jac, imgVar)
covIn = imgVar.*eye(length(jac));
varY = jac*covIn*jac';
end
