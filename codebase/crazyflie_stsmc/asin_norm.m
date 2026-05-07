function y= asin_norm(u)
if u>1
    y=pi/2;
elseif u<-1
    y=-pi/2;
else
    y=asin(u);
end