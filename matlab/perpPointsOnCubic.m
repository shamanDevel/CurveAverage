syms x0 y0 z0 x1 y1 z1 t0x t0y t0z t1x t1y t1z t qx qy qz real;
p0 = [x0; y0; z0]; p1 = [x1;y1;z1]; t0 = [t0x; t0y; t0z]; t1 = [t1x; t1y; t1z]; q = [qx;qy;qz];
pt = (2*t^3 - 3*t^2 + 1) * p0 + (t^3-2*t^2+t) * t0 + (-2*t^3+3*t^2) * p1 + (t^3-t^2) * t1;
ptv = diff(pt,t);
ft = dot(q - pt, ptv);
c = coeffs(ft,t)

