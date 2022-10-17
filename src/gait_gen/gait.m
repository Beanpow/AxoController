gait = gait1(:, [1,10,13,5,8])

in = 0:0.001:1;
hipl = spline(gait(:,1), gait(:,2), in);
kneel = spline(gait(:,1), gait(:,3), in);
hipr = spline(gait(:,1), gait(:,4), in);
kneer = spline(gait(:,1), gait(:,5), in);

kneer = kneer * 180/pi;
kneel = kneel * 180/pi