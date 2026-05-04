function u_sat = f_sat(u,umin,umax)

u_sat = u;

for i=1:size(u)
    if u(i)< umin
        u_sat(i) = umin; 
    end
    if u_sat(i) > umax
        u_sat(i) = umax;
    end
end