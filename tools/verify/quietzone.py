# Quiet-zone (white margin) sweep for 15mm ArUco on a white base.
# Paired design: identical poses + noise across all margins; only the margin changes.
import cv2, numpy as np, sys, json
L = 15.0                      # marker side mm (black outer border)
BLACK, WHITE = 22.0, 175.0    # measured on the real frame
FX = 663.0                    # from real frame: 76.5px for 15mm at ~130mm
IMG = 320
SS = 4                        # supersampling
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
MARK = cv2.aruco.generateImageMarker(d, 2, 600).astype(np.float32)  # 600px = 15mm -> 40px/mm

def make_det():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    p.aprilTagQuadDecimate = 1.0; p.aprilTagQuadSigma = 0.0
    p.aprilTagMinClusterPixels = 5; p.aprilTagMaxNmaxima = 10
    p.aprilTagCriticalRad = 10*np.pi/180; p.aprilTagMaxLineFitMse = 10.0
    p.aprilTagMinWhiteBlackDiff = 5; p.aprilTagDeglitch = 0
    p.adaptiveThreshWinSizeMin = 3; p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 4; p.adaptiveThreshConstant = 7
    p.minMarkerPerimeterRate = 0.02; p.maxMarkerPerimeterRate = 4.0
    p.polygonalApproxAccuracyRate = 0.03
    p.maxErroneousBitsInBorderRate = 0.15; p.errorCorrectionRate = 0.6
    return cv2.aruco.ArucoDetector(d, p)
DET = make_det()

def texture(margin, bg):
    """plane texture at 40px/mm, centered; covers 40mm x 40mm. bg: scalar or array."""
    S = 40; W = 40*S
    if np.isscalar(bg): t = np.full((W, W), bg, np.float32)
    else: t = bg.copy()
    b = int(round((L/2 + margin)*S)); c = W//2
    t[c-b:c+b, c-b:c+b] = WHITE
    m = BLACK + (MARK/255.0)*(WHITE-BLACK)
    t[c-300:c+300, c-300:c+300] = m
    return t, S, W

def render(tex, S, W, R, tvec, blur, noise, rng):
    K = np.array([[FX*SS, 0, IMG*SS/2], [0, FX*SS, IMG*SS/2], [0, 0, 1]])
    # plane (x,y mm) -> camera; texture px (u,v) -> plane mm: x=(u-W/2)/S, y=-(v-W/2)/S
    A = np.array([[1/S, 0, -W/2/S], [0, -1/S, W/2/S], [0, 0, 1]])
    H = K @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ A
    big = cv2.warpPerspective(tex, H, (IMG*SS, IMG*SS), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_REPLICATE)
    im = cv2.resize(big, (IMG, IMG), interpolation=cv2.INTER_AREA)
    if blur > 0: im = cv2.GaussianBlur(im, (0, 0), blur)
    im = im + rng.normal(0, noise, im.shape)
    return np.clip(np.round(im), 0, 255).astype(np.uint8)

def gt_corners(R, tvec):
    K = np.array([[FX, 0, IMG/2], [0, FX, IMG/2], [0, 0, 1]])
    obj = np.array([[-L/2, L/2, 0], [L/2, L/2, 0], [L/2, -L/2, 0], [-L/2, -L/2, 0]])
    p, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], tvec, K, None)
    return p.reshape(4, 2), obj, K

def rand_pose(rng, Z, tilt_max):
    tilt = np.deg2rad(rng.uniform(0, tilt_max)); az = rng.uniform(0, 2*np.pi)
    ax = np.array([np.cos(az), np.sin(az), 0.0])
    Rt = cv2.Rodrigues(ax*tilt)[0]
    Rz = cv2.Rodrigues(np.array([0, 0, rng.uniform(-np.pi, np.pi)]))[0]
    R = Rt @ Rz @ np.diag([1, -1, -1])     # face the camera
    t = np.array([rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), Z])
    return R, t

def run(Z, bg_kind, margins, n, tilt_max, blur, noise, seed=1):
    rng0 = np.random.default_rng(seed)
    poses = [rand_pose(rng0, Z, tilt_max) for _ in range(n)]
    seeds = rng0.integers(0, 2**31, n)
    tex_rng = np.random.default_rng(99)
    if bg_kind == 'grey': bg = 90.0
    elif bg_kind == 'dark': bg = 22.0
    else:   # clutter: random dark blobs (screw heads / holes) on grey metal
        bgimg = np.full((1600, 1600), 90.0, np.float32)
        for _ in range(60):
            cx, cy = tex_rng.integers(0, 1600, 2); r = int(tex_rng.integers(40, 160))
            cv2.circle(bgimg, (int(cx), int(cy)), r, float(tex_rng.choice([22, 45, 130])), -1)
        bg = bgimg
    res = {}
    for m in margins:
        tex, S, W = texture(m, bg)
        det_ok = 0; errs = []; bias = []; dz = []; drot = []
        for (R, t), s in zip(poses, seeds):
            rng = np.random.default_rng(int(s))
            im = render(tex, S, W, R, t, blur, noise, rng)
            c, ids, _ = DET.detectMarkers(im)
            gt, obj, K = gt_corners(R, t)
            if ids is None or 2 not in ids.ravel(): continue
            q = c[list(ids.ravel()).index(2)].reshape(4, 2)
            # match by nearest gt corner (order-agnostic)
            idx = [int(np.argmin(np.linalg.norm(gt - qq, axis=1))) for qq in q]
            if len(set(idx)) != 4: continue
            det_ok += 1
            gtm = gt[idx]
            e = q - gtm; errs.append(np.sqrt((e**2).sum(1)))
            out = gtm - gtm.mean(0); out /= np.linalg.norm(out, axis=1, keepdims=True)
            bias.append((e*out).sum(1).mean())            # + = marker looks bigger
            ok, rv, tv = cv2.solvePnP(obj[idx], q.astype(np.float64), K, None,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE if False else cv2.SOLVEPNP_IPPE)
            Re = cv2.Rodrigues(rv)[0] @ R.T
            drot.append(np.rad2deg(np.linalg.norm(cv2.Rodrigues(Re)[0])))
            dz.append(np.linalg.norm(tv.ravel()) - np.linalg.norm(t))
        errs = np.concatenate(errs) if errs else np.array([np.nan])
        res[m] = dict(rate=100*det_ok/n, cerr=float(np.sqrt(np.mean(errs**2))),
                      bias=float(np.mean(bias)) if bias else np.nan,
                      rot=float(np.sqrt(np.mean(np.square(drot)))) if drot else np.nan,
                      rot95=float(np.percentile(drot, 95)) if drot else np.nan,
                      dz=float(np.mean(dz)) if dz else np.nan)
    return res

if __name__ == '__main__':
    Z = float(sys.argv[1]); bgk = sys.argv[2]; tilt = float(sys.argv[3]); n = int(sys.argv[4])
    margins = [0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0]
    r = run(Z, bgk, margins, n, tilt, blur=0.9, noise=2.5)
    ppm = FX/Z
    print(f"== Z={Z:.0f}mm ({ppm:.2f}px/mm, module={ppm*2.5:.1f}px)  bg={bgk}  tilt<= {tilt:.0f}deg  n={n}")
    print(" margin  px    detect%  cornerRMSpx  bias_px   rotRMS   rot95    dZ_mm")
    for m, v in r.items():
        print(f"  {m:4.2f}  {m*ppm:5.1f}   {v['rate']:6.1f}   {v['cerr']:8.4f}   {v['bias']:+7.4f}  {v['rot']:6.3f}  {v['rot95']:6.3f}  {v['dz']:+7.3f}")
