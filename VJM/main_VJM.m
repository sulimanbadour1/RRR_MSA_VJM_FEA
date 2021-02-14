data = zeros(13, 13, 13);

Force = [0; 0; 100; 0; 0; 0];
pointsize = 45;
theta = zeros(1,3);
tic
for z = 1:13
    for y = 1:13
        for x = 1:13
            p_global = [x*0.1, y*0.1, z*0.1];
            Kc = Kc_def_RRR( p_global, theta);
            dt_VJM = Kc\Force;
           % dt_VJM = pinv(Kc)*Force;
            deflection = sqrt(dt_VJM(1)^2 + dt_VJM(2)^2 + dt_VJM(3)^2);
            data(x, y, z) = deflection; 
        
        scatter3(x*0.1, y*0.1, z*0.1 ,pointsize, data(x,y,z));
        hold on
        end
    end
end
toc
colorbar
