function isValid = Check2DLineStraightness(x, y, mThres)
%Checks the straightness of the data points by comparing the gradients
%between points and a fitted linear regression model.

%Author: Jasprabhjit Mehami, 13446277

%fit linear model to data
p = polyfit(x,y,1);

%extract gradient from fitted model
mFit = p(1);

%calculate gradients between consecutive points
mPoints = diff(y)./diff(x);

%calculate difference between model and point gradients
mDiff = abs(mPoints - mFit);

%if difference is greater than threshold, then data is not straight
if any(mDiff > mThres)
    isValid = false;
    return;
end

isValid = true;

end

