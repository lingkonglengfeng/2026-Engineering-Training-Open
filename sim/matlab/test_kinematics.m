function test_kinematics()
%TEST_KINEMATICS Verify the wheel layer, the nose binding and the time planner.
%
% Run:  matlab -batch "cd('E:\Engineering_Training_Project\sim\matlab'); test_kinematics"
%
% Everything here is checked against a firmware constant or an independently
% recorded fact - never against a re-derivation of the same formula.

fails = 0;
P = sim_params();

section('PARAMETERS (derived from firmware constants)');
fails = fails + expect(abs(P.mm_s_per_rpm - 4.2616) < 0.0001, ...
    'mm_s_per_rpm = 255.694226/60', sprintf('%.4f', P.mm_s_per_rpm));
fails = fails + expect(abs(P.accel_mm_s2 - 2557) < 2, ...
    'accel = 600 RPM/s -> ~2557 mm/s^2', sprintf('%.1f', P.accel_mm_s2));
fails = fails + expect(abs(P.speed_mm_s - 532.7) < 0.5, ...
    'nav speed at spd=0.50 -> ~533 mm/s', sprintf('%.1f', P.speed_mm_s));

section('MOTION PLAN: t1000 is consistent with the speed model?');
r = motion_plan(1000, P.ref_speed_mm_s, P);
fails = fails + expect(abs(r.tri_seconds * 1000 - 2050.8) < 1.0, ...
    'plan_seconds(1000mm, spd=0.50) = 2050.8 ms', sprintf('%.1f ms', r.tri_seconds * 1000));
fprintf('   t1000 default = %.1f ms  ->  k = %.4f (want ~1.000)\n', ...
    P.time_1000mm_ms, P.time_1000mm_ms / (r.tri_seconds * 1000));
fprintf('   The old 1900 ms default came from a superseded speed model and\n');
fprintf('   under-runs every segment by about 7%%.\n');

section('MOTION PLAN: what the car really does per segment (the t1000 trap)');
for v_scale = [0.50 0.75 1.00]
    Q = sim_params(struct('speed_scale', v_scale));
    for L = [150 400 850 1700]
        rr = motion_plan(L, Q.speed_mm_s, Q);
        fprintf('   spd=%.2f L=%4dmm -> drive %5.0fms, covered %7.1fmm, over %+6.1fmm (%+.2f%%)\n', ...
            v_scale, L, rr.drive_ms, rr.covered_mm, rr.overshoot_mm, ...
            100 * rr.overshoot_mm / L);
    end
end
fprintf('   The firmware commands a DURATION, not a distance, so segments\n');
fprintf('   under/over-run unless t1000 matches the speed model.\n');

section('WHEEL LAYER: forward is the exact inverse of inverse');
% Only NON-saturating twists can round trip: the forward solution reads wheel
% speeds, which carry no record that a normalization happened.
worst = 0;
for vx = [-0.5 0 0.3 0.8]
    for vy = [-0.4 0 0.6]
        for om = [-0.2 0 0.15]
            w = wheel_inverse(vx, vy, om, P);
            if w.saturated
                continue
            end
            [fx, fy, fo] = wheel_forward(w.rpm, P);
            worst = max(worst, max(abs([fx - vx, fy - vy, fo - om])));
        end
    end
end
fails = fails + expect(worst < 1e-9, ...
    'ideal round trip reconstructs (vx, vy, omega)', sprintf('max err %.2e', worst));

section('WHEEL LAYER: the firmware mix pattern');
w  = wheel_inverse(1, 0, 0, P);
w2 = wheel_inverse(0, 0, 1, P);
w3 = wheel_inverse(0, 1, 0, P);
fails = fails + expect(isequal(w.values, [1 1 1 1]), ...
    'pure forward vx=1 -> all four wheels equal', mat2str(w.values, 4));
fails = fails + expect(isequal(w2.values, [-1 1 -1 1]), ...
    'pure yaw omega=1 -> FL- FR+ RL- RR+ (MixTurn)', mat2str(w2.values, 4));
fails = fails + expect(isequal(w3.values, [1 -1 -1 1]), ...
    'pure strafe vy=1 -> FL+ FR- RL- RR+', mat2str(w3.values, 4));

% The patterns above must read BACK as pure twists, or a strafe secretly yaws.
% This is the check that a wheel_forward sign error cannot survive.
[ax, ay, ao] = wheel_forward(w3.rpm, P);
fails = fails + expect(abs(ax) < 1e-12 && abs(ay - 1) < 1e-12 && abs(ao) < 1e-12, ...
    'pure strafe reads back as (0, 1, 0) - no phantom yaw', ...
    sprintf('(%.6f, %.6f, %.6f)', ax, ay, ao));
[bx, by, bo] = wheel_forward(w2.rpm, P);
fails = fails + expect(abs(bx) < 1e-12 && abs(by) < 1e-12 && abs(bo - 1) < 1e-12, ...
    'pure yaw reads back as (0, 0, 1)', ...
    sprintf('(%.6f, %.6f, %.6f)', bx, by, bo));

section('WHEEL LAYER: saturation normalizes instead of clipping');
w = wheel_inverse(1, 1, 1, P);
% before normalization the mix is [FL FR RL RR] = [1 1 -1 3]; peak 3 -> /3
want = [1 1 -1 3] / 3;
fails = fails + expect(w.saturated && max(abs(w.values - want)) < 1e-12, ...
    'vx=vy=omega=1 -> [1 1 -1 3]/3, saturated', mat2str(w.values, 4));

section('ERROR HOOK: the phase-2 injection point is live');
[fx, fy] = wheel_forward(wheel_inverse(1, 0, 0, P).rpm, P);
[gx, gy] = wheel_forward(wheel_inverse(1, 0, 0, P).rpm, P, ...
    struct('per_wheel', [0.9 1 1 1]));
fails = fails + expect(abs(gx - fx) > 1e-6 && abs(gy - fy) > 1e-6, ...
    'a per-wheel error multiplies through (slip is injectable)', ...
    sprintf('vx %.4f -> %.4f', fx, gx));
finfo = wheel_forward(wheel_inverse(1, 0, 0, P).rpm, P);
% NOTE: the info struct is the FOURTH output of wheel_forward, not the first.
% Capturing only one output makes `info` a scalar double and the next line dies
% with "incompatible types for struct field assignment".
[~, ~, ~, winfo] = wheel_forward(wheel_inverse(1, 0, 0, P).rpm, P);
fails = fails + expect(winfo.reconstructed == false, ...
    'with no err the hook reports reconstructed=false', ...
    sprintf('%d', winfo.reconstructed));
fails = fails + expect(isfinite(finfo), 'forward returns a finite vx', ...
    sprintf('%.6f', finfo));

section('NOSE BINDING: heading vector <-> angle');
dirs = {[0 1], 'U'; [0 -1], 'D'; [1 0], 'R'; [-1 0], 'L'};
for i = 1:size(dirs, 1)
    hv = dirs{i, 1};
    c = car_init(1200, 1200, hv);
    F = [cos(c.h), sin(c.h)];
    fails = fails + expect(max(abs(F - hv)) < 1e-12, ...
        sprintf('nose [%+d %+d] (case %s) round trips', hv(1), hv(2), dirs{i, 2}), ...
        sprintf('[%.6f %.6f]', F(1), F(2)));
end

fprintf('\n%s\n', repmat('=', 1, 66));
if fails == 0
    fprintf('RESULT: PASS - wheel layer, nose binding and time planner agree\n');
else
    fprintf('RESULT: %d FAILURE(S)\n', fails);
end
fprintf('%s\n', repmat('=', 1, 66));
if fails > 0
    error('test_kinematics:failed', '%d check(s) failed', fails);
end
end

function fails = expect(ok, label, got)
if ok
    fprintf('  [ ok ] %s\n', label);
    fails = 0;
else
    fprintf('  [FAIL] %s\n         got: %s\n', label, got);
    fails = 1;
end
end

function section(title)
fprintf('\n--- %s ---\n', title);
end
