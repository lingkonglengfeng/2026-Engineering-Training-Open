function [rects, ids] = blocked_rects(spec)
%BLOCKED_RECTS Every rectangle the car BODY may never overlap, in map mm.
%
%   [rects, ids] = BLOCKED_RECTS(spec)
%
%   rects  Nx4 [x0 x1 y0 y1], map frame (origin lower-left, y UP)
%   ids    {1xN} label per row, for reporting which one was hit
%
% The single place that enumerates forbidden rectangles. CHECK_LEGALITY (runtime
% verdicts) and ARC_MIN_RADIUS (swept-arc search) both go through it, so they
% cannot disagree about what is forbidden - and cannot both be wrong in the same
% silent way.
%
% Two families:
%   BLOCKED  the four 450x450 light-yellow zones (rules p3) - no driving
%   PAD      the physical table tops / QR board - the body must never touch
% `WORK` is deliberately absent: it is only a functional-area overlay that
% contains the free stop space, and treating it as forbidden cuts the map in two.
%
% GEOMETRY - the mistake this file exists to prevent:
% The four zones form a 2x2 arrangement around the field centre. With
% lo = 550 and hi = 1850, a zone spans [lo, lo+450] on one axis and
% [hi-450, hi] on the other, i.e. the inner edges are at 1000 and 1400. Writing
% them as [lo, hi, lo, hi] instead makes four 1300x1300 rectangles that swallow
% the 400 mm cross corridor. That error made every pose "illegal", returned NaN
% from every arc-radius search, and showed up only as "this corner cannot be
% turned" - so it is written out explicitly here rather than composed inline.

fk = spec.field_kind;
lo   = fk.block_lo_mm;                     % 550
hi   = fk.block_hi_mm;                     % 1850
inlo = lo + 450;                           % 1000  inner edge, low side
inhi = hi - 450;                           % 1400  inner edge, high side

yellow = [ lo,   inlo, inhi, hi;           % north-west
           inhi, hi,   inhi, hi;           % north-east
           lo,   inlo, lo,   inlo;         % south-west
           inhi, hi,   lo,   inlo ];       % south-east

rects = [yellow; reshape(spec.pads, [], 4)];
ids = {'yellow-NW', 'yellow-NE', 'yellow-SW', 'yellow-SE', ...
       'pad-raw', 'pad-rough', 'pad-temp', 'pad-qr'};
ids = ids(1:size(rects, 1));
end
