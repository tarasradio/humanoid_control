function LIP(zc, Tsup, sx, sy, xi, yi, vxi, vyi, px, py, a, b)
% zc, Tsup: z-axis intersection; support time
% sx, sy: walk parameters(vectors)
% xi, yi: initial CoM coordinates
% px, py: initial foot placement
% vxi, vyi: initial CoM velocity
% a, b: params of costfunc
%% Initialization
n = 0;
g = 9.8;

sx = [sx, 0];
sy = [sy, 0]; % the step after the last step should be zero

px0 = px;
py0 = py; % desired foot placement

Tc = sqrt(zc / g);
C = cosh(Tsup / Tc);
S = sinh(Tsup / Tc);
D = a * (C - 1)^2 + b * (S / Tc)^2;

hold on
cal_foot_place(0); % set the initial placement of foot
%% carry out the steps indicated by walk parameters sx, sy
while n < size(sx, 2) % sx was expanded by one element previously
    %% plot the n-th step trajectory
    t = 0;
    dt = Tsup / 100;
    xt = xi; vxt = vxi; yt = yi; vyt = vyi;

    for i = 0 : 100
        xt = [xt, (xi - px) * cosh(t / Tc) +...
             Tc * vxi * sinh(t / Tc) + px];
        vxt = [vxt, (xi - px) / Tc * sinh(t / Tc) +...
              vxi * cosh(t / Tc)];
        yt = [yt, (yi - py) * cosh(t / Tc) +...
             Tc * vyi * sinh(t / Tc) + py];
        vyt = [vyt, (yi - py) / Tc * sinh(t / Tc) +...
              vyi * cosh(t / Tc)];
        t = t + dt;
    end
    plot(xt, yt);
    hold on
    %% update xi, yi, vxi, vyi for the next step
    vxi = vxt(end); vyi = vyt(end);
    xi = xt(end); yi = yt(end);
    %% update n, the order number of the step
    n = n + 1;

    if n < size(sx, 2)
        cal_foot_place(n); % calculate the actual foot place for step n
    end
end
%% calculate the foot place
function cal_foot_place(n) % nested function
    %% calculate the desired foot place during the n-th step
    if n ~= 0
        px0 = px0 + sx(n);
        py0 = py0 - (-1)^n * sy(n);
    end
    %% calculate the coordinate (xbar, ybar)
    xbar = sx(n + 1) / 2;
    ybar = (-1)^n * sy(n + 1) / 2;
    vxbar = (C + 1)/(Tc * S) * xbar;
    vybar = (C - 1)/(Tc * S) * ybar;
    %% target state of CoM, of the n-th step
    xd = px0 + xbar;
    yd = py0 + ybar;
    vxd = vxbar;
    vyd = vybar;
    %% update px, py to be the real foot place in step n
    % a,b are parameters for cost func
    px = - a * (C - 1) / D * (xd - C * xi - Tc * S * vxi)...
        - b * S / (Tc * D) * (vxd - S / Tc * xi - C * vxi);
    py = - a * (C - 1) / D * (yd - C * yi - Tc * S * vyi)...
        - b * S / (Tc * D) * (vyd - S / Tc * yi - C * vyi);
    plot(px0, py0, 'x');
    plot(px, py, 'o');
end
end