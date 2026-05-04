function y= asin_norm(u)
if u>1
    y=pi/2;
elseif u<-1
    y=-pi/2;
else
    y=asin(u);
end

% function y = asin_norm(u)
%     u = real(u);                 % ensure real
%     u = max(-1, min(1, u));      % clamp safely
%     y = asin(u);
% end