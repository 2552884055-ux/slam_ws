#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 显式系统解释器, 避免 pyenv/conda 截走
"""
extend_pcd.py —— 把真实管廊点云沿主轴(X)平铺延长到指定长度

做法:
  1. 解析 PCD(8字段: x y z intensity nx ny nz curvature), 去离群;
  2. 取一段"干净的隧道中段"作为重复单元(保留真实断面/特征);
  3. 沿 X 平铺 N 次拼到目标长度, 每块加小幅 xyz 微扰, 避免完全周期化(更真实);
  4. 写出新的二进制 PCD。

用法:
  /usr/bin/python3 extend_pcd.py --in scans.pcd --out scans_100m.pcd --length 100
"""
import argparse
import numpy as np

FIELDS = "x y z intensity normal_x normal_y normal_z curvature"


def load_pcd8(path):
    with open(path, 'rb') as f:
        head = f.read(512)
    off = head.find(b'DATA binary\n') + len(b'DATA binary\n')
    with open(path, 'rb') as f:
        f.seek(off); data = np.fromfile(f, dtype=np.float32)
    n = data.size // 8
    return data[:n * 8].reshape(n, 8)


def write_pcd8(path, arr):
    n = len(arr)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {FIELDS}\n"
        "SIZE 4 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        arr.astype(np.float32).tofile(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--length", type=float, default=100.0)   # 目标长度(m)
    ap.add_argument("--ref_x0", type=float, default=10.0)    # 参考段起点(原始坐标X, 该段覆盖完整)
    ap.add_argument("--ref_x1", type=float, default=20.0)    # 参考段终点
    ap.add_argument("--profile_thickness", type=float, default=1.5)  # 参考段中心取的薄切片厚度(m)
    ap.add_argument("--symmetrize", type=int, default=0)     # 1: 沿隧道中线镜像补全(单侧缺失时用)
    ap.add_argument("--declutter", type=int, default=1)      # 1: 去掉拱内悬浮杂点(只留地面+贴壁轮廓)
    ap.add_argument("--floor_band", type=float, default=0.35)  # 地面带厚度(m), 全保留
    ap.add_argument("--wall_margin", type=float, default=0.5)  # 贴壁保留宽度(m)
    # 隧道管裁剪(相对地面Z的高度窗 + 相对中心Y的半宽, 去上方离群/侧边结构)
    ap.add_argument("--tunnel_halfw", type=float, default=5.0)   # 横向半宽(m)
    ap.add_argument("--z_lo", type=float, default=-1.0)          # 地面下留余量
    ap.add_argument("--z_hi", type=float, default=6.0)           # 地面上净高窗
    ap.add_argument("--jitter", type=float, default=0.015)   # 扫掠微噪 std(m)
    ap.add_argument("--density", type=float, default=265000)  # 每米点数(默认≈真实密度)
    ap.add_argument("--profile_max", type=int, default=500000)  # 断面保留点数上限(分辨率)
    ap.add_argument("--feature_spacing", type=float, default=0.0)  # >0: 每隔Nm放地面杂物簇(可调退化强度)
    ap.add_argument("--max_pts", type=int, default=600000)   # 兼容保留
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    arr = load_pcd8(a.inp)
    xyz = arr[:, :3]
    ok = np.isfinite(arr).all(1) & (np.abs(xyz) < 500).all(1)
    arr = arr[ok]; xyz = arr[:, :3]

    # ---- 1) 裁掉离群只留隧道管 (原始坐标; 生切片本就是完整拱形, 含离群点的全局PCA会转歪坐标系) ----
    zc = np.percentile(xyz[:, 2], 5)                    # 地面高度
    bulk = xyz[xyz[:, 2] < zc + 6.0]                    # 隧道主体(排除上方离群)
    yc = np.median(bulk[:, 1])                          # 隧道横向中心
    keep = ((xyz[:, 2] > zc + a.z_lo) & (xyz[:, 2] < zc + a.z_hi) &
            (np.abs(xyz[:, 1] - yc) < a.tunnel_halfw))
    arr = arr[keep]
    print("[extend] 裁隧道管: 中心Y=%.1f 地面Z=%.1f, 余 %d 点" % (yc, zc, len(arr)))

    # ---- 2) 在参考段中心(原始X)取薄切片做断面 (隧道轴~7°倾斜对薄切片影响可忽略) ----
    xmid = 0.5 * (a.ref_x0 + a.ref_x1)
    slab = arr[np.abs(arr[:, 0] - xmid) < a.profile_thickness / 2.0]
    prof = slab[:, 1:3].copy()                          # (Y,Z) 真实完整拱形断面
    print("[extend] 参考断面 原始X=%.1f±%.1fm, %d 点" % (xmid, a.profile_thickness / 2, len(prof)))
    # 真实扫描常单侧缺失 -> 沿隧道中线镜像补全, 闭合成对称完整拱形
    # 中线由「地面」决定(地面全宽扫到, 不偏); 用最低Z带点的Y跨度中点, 而非全体中位(会被单侧覆盖带偏)
    if a.symmetrize:
        # 对称轴 = 拱顶顶点的 Y (最高Z带的中位Y)
        ztop = np.percentile(prof[:, 1], 90)
        crown = prof[prof[:, 1] > ztop]
        yc_p = np.median(crown[:, 0]) if len(crown) > 20 else np.median(prof[:, 0])
        # 只取「覆盖完整的那半边」+ 它的镜像 -> 每侧单线, 不会双线/错位
        left, right = prof[prof[:, 0] <= yc_p], prof[prof[:, 0] > yc_p]
        half = left if len(left) >= len(right) else right
        mir = half.copy(); mir[:, 0] = 2 * yc_p - mir[:, 0]
        prof = np.concatenate([half, mir], 0)
        print("[extend] 镜像中线(拱顶) Y=%.2f, 取%s半边镜像补全" % (yc_p, "左" if len(left) >= len(right) else "右"))
    # 去掉拱内悬浮杂点: 保留地面带 + 每个高度层只留贴左右壁的点 (中间悬空点删除)
    if a.declutter:
        z0 = np.percentile(prof[:, 1], 1)
        keep = prof[:, 1] < z0 + a.floor_band          # 地面带全留
        zb = np.arange(z0 + a.floor_band, prof[:, 1].max() + 0.25, 0.25)
        for z in zb:
            m = (prof[:, 1] >= z) & (prof[:, 1] < z + 0.25)
            if m.sum() < 5:
                continue
            ylo, yhi = prof[m, 0].min(), prof[m, 0].max()
            near = (np.abs(prof[:, 0] - ylo) < a.wall_margin) | (np.abs(prof[:, 0] - yhi) < a.wall_margin)
            keep |= (m & near)
        n0 = len(prof); prof = prof[keep]
        print("[extend] 去杂点: %d -> %d (删 %d 悬浮点)" % (n0, len(prof), n0 - len(prof)))
    if len(prof) > a.profile_max:
        prof = prof[rng.choice(len(prof), a.profile_max, replace=False)]
    print("[extend] 断面 %d 点%s; 沿轴扫掠到 %.1fm" %
          (len(prof), " (+镜像补全)" if a.symmetrize else "", a.length))

    # ---- 4) 沿 X 连续「扫掠」断面 -> 无缝平滑的 100m 隧道 (断面真实, 纵向连续虚构) ----
    n_pts = int(a.density * a.length)
    pidx = rng.integers(0, len(prof), n_pts)
    X = rng.uniform(0.0, a.length, n_pts)          # 连续均匀分布 -> 无缝
    YZ = prof[pidx]
    base = np.zeros((n_pts, 8), dtype=np.float32)
    base[:, 0] = X
    base[:, 1:3] = YZ
    base[:, :3] += rng.normal(0, a.jitter, (n_pts, 3))  # 自然微噪

    parts = [base]
    # 可选: 沿轴等间隔放置真实地面杂物簇, 提供间歇性纵向特征(更真实, 可调退化强度)
    # prof 列: [Y, Z]; yc=中心Y, zc=地面Z
    if a.feature_spacing > 0:
        floor_cand = prof[(prof[:, 0] > yc - 2.0) & (prof[:, 0] < yc + 2.0) & (prof[:, 1] < zc + 1.2)]
        if len(floor_cand) > 50:
            xs_f = np.arange(5.0, a.length - 2.0, a.feature_spacing)
            feats = []
            for xf in xs_f:
                k = min(2500, len(floor_cand))
                cl = floor_cand[rng.choice(len(floor_cand), k, replace=False)]
                b = np.zeros((k, 8), dtype=np.float32)
                b[:, 0] = xf + rng.normal(0, 0.25, k)       # 纵向窄簇 -> 局部X特征
                b[:, 1:3] = cl + rng.normal(0, 0.02, (k, 2))
                feats.append(b)
            parts.append(np.concatenate(feats, 0))
            print("[extend] 加入 %d 处纵向杂物特征 (间隔 %.1fm)" % (len(xs_f), a.feature_spacing))

    out = np.concatenate(parts, 0)
    out = out[(out[:, 0] >= 0) & (out[:, 0] <= a.length)]
    write_pcd8(a.out, out)
    mn, mx = out[:, :3].min(0), out[:, :3].max(0)
    print("[extend] 写出 %s : %d 点, X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"
          % (a.out, len(out), mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))


if __name__ == "__main__":
    main()
