function diff = angleDiff(a, b)
%ANGLEDIFF Signed angular difference (a - b) normalised to [-pi, +pi].
%
%  Correctly handles wrap-around at +/-pi.
%
%  diff = angleDiff(a, b)
%
%  Example:
%    angleDiff(pi - 0.1,  -pi + 0.1)  →  -0.2   (short path through +/-pi)
%    angleDiff(0.1, -0.1)             →   0.2

diff = mod(a - b + pi, 2*pi) - pi;
end
