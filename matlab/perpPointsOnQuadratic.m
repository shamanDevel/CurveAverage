syms x0 y0 z0 x1 y1 z1 t0x t0y t0z t qx qy qz real;
p0 = [x0; y0; z0]; p1 = [x1;y1;z1]; t0 = [t0x; t0y; t0z]; q = [qx;qy;qz];
pt = (t^2 - 2*t + 1) * p0 + (-t^2 + 2*t) * p1 + (t^2 - t) * t0;
ptv = diff(pt,t);
ft = dot(q - pt, ptv);
c = coeffs(ft,t)

