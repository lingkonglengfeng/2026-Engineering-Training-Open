"""放料后圆环：轮廓只提供搜索初值，圆心必须由当前多层外环暗线重测。

由 sync_occupied_ring.py 嵌入 middle_ring.py，设备仍可单文件上传。
不使用物料顶面圆心，也不把历史位置或等距中点当成测量结果。
"""
import math
import cv2
import numpy as np
import time


def occupied_sample_group(gray, seed, normalize, group_layers, fit_circle, refined=False, sampled=None):
    started=time.monotonic()
    occupied_sample_group.last_profile=None
    cx, cy = seed['center']
    radius = seed.get('sampling_radius', seed['radius'])
    h, w = gray.shape
    if not 12 <= radius <= min(h, w)*.425:
        return None
    if not (-.65*radius <= cx <= w-1+.65*radius and -.65*radius <= cy <= h-1+.65*radius):
        return None
    _, inverse, ratios, angle, extents = normalize(seed)
    cropped = cx-radius*extents[0] < 0 or cx+radius*extents[0] >= w or cy-radius*extents[1] < 0 or cy+radius*extents[1] >= h
    boundary_context=bool(seed.get('boundary_verified')) and sum((cx-radius*extents[0]<0,cx+radius*extents[0]>=w,cy-radius*extents[1]<0,cy+radius*extents[1]>=h))==1
    if sampled is None:
        angles,radial,nx,ny,mx,my=occupied_sampling_grid(seed,normalize)
        values=cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    else:
        angles,radial,nx,ny,mx,my,values=sampled
    ray_scale=len(angles)/90
    mapped=time.monotonic()
    # 保证暗线对比窗口全在相机内，禁止把 remap 黑边当成圆弧。
    inside = ((mx > 2) & (mx < w-3) & (my > 2) & (my < h-3)).astype(np.uint8)
    kernel=getattr(occupied_sample_group,'valid_kernel',None)
    if kernel is None:
        kernel=np.ones((1,13),np.uint8);occupied_sample_group.valid_kernel=kernel
    valid = cv2.erode(inside, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    smooth = cv2.blur(np.vstack((values[-1:], values, values[:1])), (1, 3))[1:-1]
    noise = occupied_median(np.abs(values-smooth)[valid])/.55 if valid.any() else 0.0
    values = smooth
    dip = cv2.blur(values, (13, 1))-values
    dip[~valid] = -1000
    signaled=time.monotonic()
    candidates = []
    row_cache=getattr(occupied_sample_group,'row_cache',None)
    if row_cache is None:
        row_cache={};occupied_sample_group.row_cache=row_cache
    if len(angles) not in row_cache:row_cache[len(angles)]=np.arange(len(angles))
    rows=row_cache[len(angles)]
    for layer, fraction in enumerate((65/95, 75/95, 85/95, 1.0)):
        band = np.flatnonzero(np.abs(radial-fraction*radius) <= max(1.5, .045*radius))
        if len(band) < 3:
            continue
        available = valid[:, band].any(axis=1)
        possible = int(available.sum())
        if possible < math.ceil(((24 if boundary_context else 28) if cropped else 90)*ray_scale):
            continue
        indices = band[np.argmax(dip[:, band], axis=1)]
        strength = dip[rows, indices]
        selected = (strength >= max(5, 2*noise)) & available
        support = .70 if cropped else .60
        minimum = max(math.ceil((22 if boundary_context else 26)*ray_scale), math.ceil(possible*(.65 if cropped else .55)))
        if int(selected.sum()) < possible*support or occupied_median(strength[selected]) < max(8, 3*noise):
            continue
        sx, sy = nx[rows, indices][selected], ny[rows, indices][selected]
        active = np.ones(len(sx), dtype=bool)
        fitted = None
        for iteration in range(3):
            fitted = fit_circle(sx[active], sy[active])
            if fitted is None:
                break
            fx, fy, fitted_radius = fitted
            absolute = np.abs(np.hypot(sx-fx, sy-fy)-fitted_radius)
            if iteration == 2:
                break
            updated = absolute <= max(.7, min(1.5, 3*occupied_median(absolute[active])))
            if int(updated.sum()) < minimum:
                break
            if np.array_equal(updated,active):
                break  # 内点集合不再变化；重复解相同方程不会改变结果。
            active = updated
        if fitted is None or int(active.sum()) < minimum:
            continue
        fx, fy, fitted_radius = fitted
        residual = np.abs(np.hypot(sx[active]-fx, sy[active]-fy)/fitted_radius-1)
        center = (cx+inverse[0]*fx+inverse[1]*fy, cy+inverse[2]*fx+inverse[3]*fy)
        error = float(residual.mean())
        # 覆盖角按实际有效点最大缺口计算，而非用离散杂点冒充一圈。
        # Sampling angles and boolean indexing already preserve ascending order.
        supported_angles = angles[selected][active]
        gap=float((supported_angles[1:]-supported_angles[:-1]).max()) if len(supported_angles)>1 else 0.
        wrap=float(supported_angles[0]+2*math.pi-supported_angles[-1])
        coverage = math.degrees(2*math.pi-max(gap,wrap))
        measured_crop = (center[0]-fitted_radius*extents[0] < 0 or center[0]+fitted_radius*extents[0] >= w
                         or center[1]-fitted_radius*extents[1] < 0 or center[1]+fitted_radius*extents[1] >= h)
        p95=float(np.partition(residual,math.ceil(.95*(len(residual)-1)))[math.ceil(.95*(len(residual)-1))])
        if (error > .025 or p95 > .055
                or math.dist(center, (cx, cy)) > max(4, radius*(.25 if seed.get('coarse') else .12))
                or abs(fitted_radius-fraction*radius) > radius*.07
                or coverage < ((84 if boundary_context else 100) if measured_crop else 210)
                or (not measured_crop and (int(selected.sum()) < math.ceil(54*ray_scale) or int(active.sum()) < math.ceil(50*ray_scale)))):
            continue
        candidates.append(dict(center=center, radius=fitted_radius, error=error, source=layer, sample_fraction=fraction,
            partial=measured_crop, arc_deg=coverage,
            ellipse=(center, (2*fitted_radius*ratios[0], 2*fitted_radius*ratios[1]), angle)))
    fitted_at=time.monotonic()
    occupied_sample_group.last_profile=dict(map=1000*(mapped-started),signal=1000*(signaled-mapped),fit=1000*(fitted_at-signaled))
    recovered = group_layers(candidates)
    if len(recovered)!=1 and boundary_context:
        # 仅已由两个当前完整环确定第三组位置时，允许边界外圈85/95两层证明该组。
        # 一条粗线的内外边缘不能冒充两层；普通独立单双圈仍由原三层门槛拒绝。
        outer_layers=[c for c in candidates if c['source'] in (2,3) and c['partial']]
        if len(outer_layers)==2:
            inner,outer=sorted(outer_layers,key=lambda c:c['radius'])
            if (.86 <= inner['radius']/outer['radius'] <= .93
                    and math.dist(inner['center'],outer['center']) <= max(1.5,.035*radius)
                    and math.dist(outer['center'],(cx,cy)) <= max(3,.06*radius)
                    and .93 <= outer['radius']/radius <= 1.07):
                center=tuple((inner['center'][i]+outer['center'][i])/2 for i in (0,1))
                recovered=[dict(center=center,radius=outer['radius'],levels=2,
                    ellipse=(center,outer['ellipse'][1],outer['ellipse'][2]),score=.80,
                    partial=True,boundary_verified=True)]
    if len(recovered) != 1:
        if candidates and not refined:
            new = dict(seed)
            new['center'] = max(candidates, key=lambda c: c['radius'])['center']
            return occupied_sample_group(gray, new, normalize, group_layers, fit_circle, True)
        return None
    result = recovered[0]
    rx, ry = result['center']
    outer = result['radius']
    if not (-.65*outer <= rx <= w-1+.65*outer and -.65*outer <= ry <= h-1+.65*outer):
        return None
    result['occluded'] = True
    # 外层暂时失去对比时，不能把下一层误作新的物理外径，逐帧缩小搜索圈。
    result['sampling_radius'] = occupied_median([c['radius']/c['sample_fraction'] for c in candidates])
    return result


def fit_occupied_circle(x,y):
    """Same centered2x2 least-squares circle; one3x3 Gram also supplies RHS/meanZ.

    Occupied path only. Retains all samples/iterations, avoids a second GEMM
    and final mean call; no SVD or changed geometric thresholds.
    """
    if len(x)<12:return None
    points=np.column_stack((x,y)).astype(np.float32,copy=False)
    mean_x,mean_y=cv2.mean(points.reshape(-1,1,2))[:2]
    centered=points-np.asarray((mean_x,mean_y),dtype=np.float32)
    zz=cv2.reduce(centered*centered,1,cv2.REDUCE_SUM)
    basis=np.column_stack((centered,zz))
    gram=cv2.gemm(basis,basis,1.,None,0.,flags=cv2.GEMM_1_T)
    aa,bb,cc=float(gram[0,0]),float(gram[0,1]),float(gram[1,1])
    dd,ee=.5*float(gram[0,2]),.5*float(gram[1,2])
    determinant=aa*cc-bb*bb
    if aa<=0 or cc<=0 or determinant<=1e-8*aa*cc:return None
    ox,oy=(dd*cc-bb*ee)/determinant,(aa*ee-bb*dd)/determinant
    radius=math.sqrt(ox*ox+oy*oy+(aa+cc)/len(x))
    return mean_x+ox,mean_y+oy,radius


def recover_occupied_rings(gray, contours, groups, normalize, group_layers, fit_circle, clipped_arcs, roi=None, hints=None):
    """仅在空环路径不能确认三环时调用；拟合/采样次数有界。

    输入已经提取的轮廓，不重复全图二值化。粗椭圆不具有目标资格。
    """
    h, w = gray.shape
    if hints:
        measured = list(groups)
        for hint in hints[:3]:
            if any(math.dist(hint['center'], g['center']) < .3*hint['radius'] for g in measured):
                continue
            current = occupied_sample_group(gray, hint, normalize, group_layers, fit_circle)
            if current is not None:
                measured.append(current)
        if len(measured) == 3:
            return measured  # 仍需调用方检查三组共线/等距/唯一身份。
    seeds = []
    border = []
    # C形薄轮廓面积小，但长度足够；最长轮廓优先且最多32次自由椭圆拟合。
    for contour, bounds, _ in sorted(contours, key=lambda m: len(m[0]), reverse=True)[:32]:
        x, y, bw, bh = bounds
        if len(contour) < 70 or min(bw, bh) < 18 or max(bw, bh) > min(h, w)*.85:
            continue
        if x <= 2 or y <= 2 or x+bw >= w-2 or y+bh >= h-2:
            border.append((contour,bounds))
            continue
        points = contour[::max(1, math.ceil(len(contour)/100))].astype(np.float32)
        ellipse = cv2.fitEllipse(points)
        center, axes, _ = ellipse
        if (min(axes) < 24 or max(axes) > min(h, w)*.85 or min(axes)/max(axes) < .72
                or not all(math.isfinite(v) for v in (*center, *axes))):
            continue
        seeds.append(dict(center=center, radius=sum(axes)/4, ellipse=ellipse))
    # 当前完整外圈提供椭圆比例；画面缩放/相机投影不必强行当成正圆。
    round_seeds=[s for s in seeds if min(s['ellipse'][1])/max(s['ellipse'][1]) >= .9]
    reference = groups[0] if groups else max(round_seeds or seeds,key=lambda s:s['radius']) if seeds else None
    board_reference=reference
    for contour,bounds in border:
        for arc in clipped_arcs(contour, gray.shape, reference, bounds=bounds):
            if arc['radius'] >= 12:
                seeds.append(arc)
    clusters = []
    for seed in sorted(seeds, key=lambda s: s['radius'], reverse=True):
        cluster = next((c for c in clusters if math.dist(c[0]['center'], seed['center']) < .30*max(c[0]['radius'], seed['radius'])), None)
        if cluster is None:
            clusters.append([seed])
        else:
            cluster.append(seed)
    recovered = list(groups)
    for cluster in clusters[:6]:
        maximum = max(s['radius'] for s in cluster)
        # 防止较大的物料轮廓拉偏初值：优先采用圆形的同心外环候选。
        circular = [s for s in cluster if s['radius'] >= maximum*.65 and min(s['ellipse'][1])/max(s['ellipse'][1]) >= .90]
        good = circular or cluster
        cx = occupied_median([s['center'][0] for s in good])
        cy = occupied_median([s['center'][1] for s in good])
        if any(math.dist((cx, cy), g['center']) < .30*maximum and g['radius'] >= maximum*.9 for g in groups):
            continue
        reference = board_reference or max(good, key=lambda s: s['radius'])
        trial = []
        for factor in (1.0, .88, 1.08, 1.16):
            seed = dict(reference, center=(cx, cy), radius=maximum*factor, coarse=True)
            # 搜索ROI人为切开的圆不能作为相机残弧。
            if roi is not None:
                left, top, right, bottom = roi
                _, _, _, _, extents = normalize(seed)
                rx, ry = seed['radius']*extents[0], seed['radius']*extents[1]
                if ((left > 0 and cx-rx <= left+2) or (top > 0 and cy-ry <= top+2)
                        or (right < w and cx+rx >= right-2) or (bottom < h and cy+ry >= bottom-2)):
                    continue
            current = occupied_sample_group(gray, seed, normalize, group_layers, fit_circle)
            if current is not None:
                trial.append(current)
                break  # 已由本帧测齐多层，无需再试所有尺度寻找更大的圈。
        if trial:
            best = max(trial, key=lambda g: g['radius'])
            matches = [g for g in recovered if math.dist(g['center'], best['center']) < .30*max(g['radius'], best['radius'])]
            if not matches:
                recovered.append(best)
            elif all(best['radius'] > g['radius']*1.05 for g in matches):
                recovered = [g for g in recovered if g not in matches]+[best]
    # 透过原始边界弧给搜索初值，当前中环必须实际测到三层，绝不输出中点。
    if len(recovered)==1:
        anchor=recovered[0]
        for edge in (s for s in seeds if s.get('partial')):
            distance=math.dist(anchor['center'],edge['center'])
            if not 4.5 <= distance/max(anchor['radius'],1) <= 9:
                continue
            center=tuple((anchor['center'][i]+edge['center'][i])/2 for i in (0,1))
            seed=dict(anchor,center=center,radius=distance*95/600)
            seed.pop('sampling_radius',None)
            measured=occupied_sample_group(gray,seed,normalize,group_layers,fit_circle)
            if measured is not None:
                recovered.append(measured)
                break
    if len(recovered)==2:
        first,last=recovered
        distance=math.dist(first['center'],last['center'])
        ratio=distance/max((first['radius']+last['radius'])/2,1)
        targets=[]
        if 5<=ratio<=8:
            targets.append((tuple((first['center'][i]+last['center'][i])/2 for i in (0,1)),distance*95/600,False))
        elif 2.2<=ratio<=4.8 and not any(g.get('partial') for g in recovered):
            for a,b in ((first,last),(last,first)):
                targets.append((tuple(2*a['center'][i]-b['center'][i] for i in (0,1)),distance*95/300,True))
        for center,radius,boundary in targets:
            if boundary and not (center[0]<0 or center[0]>=w or center[1]<0 or center[1]>=h):
                continue
            seed=dict(first,center=center,radius=radius,boundary_verified=boundary)
            seed.pop('sampling_radius',None)
            measured=occupied_sample_group(gray,seed,normalize,group_layers,fit_circle)
            if measured is not None:
                recovered.append(measured)
    return sorted(recovered, key=lambda g: g['score'], reverse=True)[:8]


def occupied_median(values):
    """小阵列用直接partition求同样的中位数，省去通用统计函数的归约调度。"""
    array=np.asarray(values).reshape(-1)
    count=len(array)
    if count==0:
        return 0.0
    middle=count//2
    if count%2:
        return float(np.partition(array,middle)[middle])
    split=np.partition(array,(middle-1,middle))
    return (float(split[middle-1])+float(split[middle]))/2


def occupied_sampling_grid(seed,normalize,step=4):
    """相同角度只计算一次；采样阵列保持float32。"""
    cache=getattr(occupied_sampling_grid,'polar',None)
    if cache is None:
        cache={}
        occupied_sampling_grid.polar=cache
    if step not in cache:
        angles=np.arange(0,360,step,dtype=np.float32)*(math.pi/180)
        cache[step]=(angles,np.cos(angles)[:,None],np.sin(angles)[:,None])
    angles,unit_x,unit_y=cache[step]
    cx,cy=seed['center']
    radius=seed.get('sampling_radius',seed['radius'])
    inverse=normalize(seed)[1]
    radial=np.arange(.62*radius,1.07*radius,max(.3,radius/110),dtype=np.float32)
    nx,ny=unit_x*radial,unit_y*radial
    mx=(cx+inverse[0]*nx+inverse[1]*ny).astype(np.float32)
    my=(cy+inverse[2]*nx+inverse[3]*ny).astype(np.float32)
    return angles,radial,nx,ny,mx,my


def occupied_sample_batch(gray,seeds,normalize):
    """将三组的采样映射拼成一个窄矩阵，只调用一次remap。

    已确认、画面内的环先用60方向；边界仍90方向。失败时由调用方回到90方向。
    """
    started=time.monotonic()
    grids=[]
    width=0
    for seed in seeds:
        cx,cy=seed['center']
        r=seed.get('sampling_radius',seed['radius'])
        extents=normalize(seed)[4]
        h,w=gray.shape
        inside=cx-r*extents[0]>=3 and cx+r*extents[0]<w-3 and cy-r*extents[1]>=3 and cy+r*extents[1]<h-3
        grid=occupied_sampling_grid(seed,normalize,6 if inside else 4)
        grids.append(grid)
        width=max(width,grid[-1].shape[1])
    height=sum(len(g[0]) for g in grids)
    mx=np.full((height,width),-1000,dtype=np.float32)
    my=np.full_like(mx,-1000)
    offset=0
    for grid in grids:
        count,cols=grid[-1].shape
        mx[offset:offset+count,:cols]=grid[-2]
        my[offset:offset+count,:cols]=grid[-1]
        offset+=count
    values=cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    result=[]
    offset=0
    for grid in grids:
        count,cols=grid[-1].shape
        result.append((*grid,values[offset:offset+count,:cols]))
        offset+=count
    occupied_sample_batch.last_profile=dict(ms=1000*(time.monotonic()-started),rays=height)
    return result
