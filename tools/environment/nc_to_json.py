# encoding: utf-8
"""
nc -> NavigationDemo JSON 预处理。
将下载的 *_cur.nc / *_wave.nc / *_wind.nc 转成 NavigationDemo 可直接读取的 JSON：
  {projectName, area{lonMin,lonMax,latMin,latMax}, gridXY{nx,ny}, <扁平 List, 行主序 ny*nx>}
  wind -> windDataU/windDataV ; cur -> currentDataU/currentDataV ; wave -> waveDataD/waveDataP/waveDataH

关键：
  - 目标网格 lon=linspace(lonMin,lonMax,nx)，lat=linspace(latMin,latMax,ny)，与 exampleData 口径一致。
  - lat 行向（i=0 → 北 latMax 还是 南 latMin）通过 --validate 对拍 exampleData 自动判定并定死。
  - Windows 中文路径：先把 nc 拷到 ASCII 临时目录再用 xarray 打开（netCDF4 C 库限制）。
  - 缺测(NaN)输出为 JSON null（与 NavigationDemo to2DArray 的 null->NaN 对齐）。

用法:
  # 对拍验证（用现成 ncData/20260311 生成并与 exampleData/20260311 逐格比对，判定 lat 行向）
  python nc_to_json.py --validate
  # 正式转换某日
  python nc_to_json.py --nc-dir <dir> --date 20260601 --out-dir <dir> --lat-order desc
"""
import os, sys, json, shutil, tempfile, argparse, math
import numpy as np
import xarray as xr

BASE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.abspath(os.environ.get('ST_DFSO_ENV_ROOT', 'data/environment'))

# nc 变量名 -> JSON 字段（带别名兜底）
FIELDS = {
    "cur":  [("currentDataU", ["uo", "utotal", "eastward_sea_water_velocity", "u"]),
             ("currentDataV", ["vo", "vtotal", "northward_sea_water_velocity", "v"])],
    "wave": [("waveDataH", ["VHM0", "swh", "significant_wave_height"]),
             ("waveDataP", ["VTPK", "VTM10", "pp1d", "peak_wave_period"]),
             ("waveDataD", ["VMDR", "mwd", "mean_wave_direction"])],
    "wind": [("windDataU", ["u10", "10u", "eastward_wind", "u"]),
             ("windDataV", ["v10", "10v", "northward_wind", "v"])],
}


def open_nc(path):
    tmp = os.path.join(tempfile.gettempdir(), "nc2json_tmp.nc")
    shutil.copyfile(path, tmp)
    # decode_timedelta=False：规避 CDS 风场 forecast_period(timedelta) 的 dtype 冲突
    # （xarray 2025.x 更严格；我们后面会 isel 掉该维，不需要它被解码）
    return xr.open_dataset(tmp, decode_timedelta=False)


def coord_name(ds, cands):
    for n in cands:
        if n in ds.variables or n in ds.dims:
            return n
    raise KeyError("找不到坐标 %s，现有=%s" % (cands, list(ds.coords)))


def pick_var(ds, aliases):
    for a in aliases:
        if a in ds.variables:
            return a
    raise KeyError("找不到变量 %s，现有=%s" % (aliases, list(ds.data_vars)))


def field_2d_ascending(ds, varname, tlats, tlons):
    """取变量 -> 降维成 (lat,lon) -> 插值到目标网格；返回 shape (ny,nx)，lat/lon 均升序。"""
    latn = coord_name(ds, ["latitude", "lat", "nav_lat", "y"])
    lonn = coord_name(ds, ["longitude", "lon", "nav_lon", "x"])
    da = ds[varname]
    sel = {d: 0 for d in da.dims if d not in (latn, lonn)}
    if sel:
        da = da.isel(sel)
    da = da.rename({latn: "_lat", lonn: "_lon"}).sortby("_lat").sortby("_lon")
    out = da.interp(_lat=tlats, _lon=tlons, method="linear")
    return np.asarray(out.values, dtype=float)  # (ny, nx), 行=升序 lat, 列=升序 lon


def flatten(vals_asc, lat_order, lon_order, ndigits=4):
    a = vals_asc
    if lat_order == "desc":
        a = a[::-1, :]
    if lon_order == "desc":
        a = a[:, ::-1]
    flat = []
    ny, nx = a.shape
    for i in range(ny):
        for j in range(nx):
            v = a[i, j]
            flat.append(None if not np.isfinite(v) else round(float(v), ndigits))
    return flat


def build(nc_dir, date, area, nx, ny):
    """读三种 nc，返回 {kind: {field: vals_asc(ny,nx)}}。"""
    tlats = np.linspace(area["latMin"], area["latMax"], ny)
    tlons = np.linspace(area["lonMin"], area["lonMax"], nx)
    out = {}
    for kind, fmap in FIELDS.items():
        path = os.path.join(nc_dir, "%s00_%s.nc" % (date, kind))
        if not os.path.exists(path):
            print("  [skip] 缺文件 %s" % os.path.basename(path))
            continue
        ds = open_nc(path)
        out[kind] = {}
        for field, aliases in fmap:
            var = pick_var(ds, aliases)
            out[kind][field] = field_2d_ascending(ds, var, tlats, tlons)
        ds.close()
    return out


def load_example_field(example_dir, date, kind, field):
    p = os.path.join(example_dir, "%s_%s.json" % (date, kind))
    with open(p, encoding="utf-8") as f:
        d = json.load(f)
    return d[field]


def _num(v):
    """安全转 float：None / 非数值字符串(如 "NaN") / 非有限 → None。"""
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def diff_flat(a, b):
    n = s = mx = 0
    for x, y in zip(a, b):
        fx = _num(x); fy = _num(y)
        if fx is None or fy is None:
            continue
        e = abs(fx - fy); s += e; mx = max(mx, e); n += 1
    return (s / n if n else float("nan")), mx, n


def write_json(out_dir, date, area, nx, ny, kind, fields_flat):
    os.makedirs(out_dir, exist_ok=True)
    obj = {"projectName": date,
           "area": {"lonMin": area["lonMin"], "lonMax": area["lonMax"],
                    "latMin": area["latMin"], "latMax": area["latMax"]},
           "gridXY": {"nx": nx, "ny": ny}}
    obj.update(fields_flat)
    p = os.path.join(out_dir, "%s_%s.json" % (date, kind))
    with open(p, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, allow_nan=False)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nc-dir", default=os.path.join(DEMO, "ncData", "20260311"))
    ap.add_argument("--date", default="20260311")
    ap.add_argument("--out-dir", default=os.path.join(BASE, "_json_out"))
    ap.add_argument("--example-dir", default=os.path.join(DEMO, "exampleData"))
    ap.add_argument("--lonMin", type=float, default=114.0)
    ap.add_argument("--lonMax", type=float, default=125.0)
    ap.add_argument("--latMin", type=float, default=19.0)
    ap.add_argument("--latMax", type=float, default=29.0)
    ap.add_argument("--nx", type=int, default=132)
    ap.add_argument("--ny", type=int, default=120)
    ap.add_argument("--lat-order", default="auto", choices=["auto", "asc", "desc"])
    ap.add_argument("--lon-order", default="asc", choices=["asc", "desc"])
    ap.add_argument("--out-flat", action="store_true", help="直接写到 out-dir，不建 {date} 子目录(时间序列同目录)")
    ap.add_argument("--validate", action="store_true")
    args = ap.parse_args()

    area = {"lonMin": args.lonMin, "lonMax": args.lonMax, "latMin": args.latMin, "latMax": args.latMax}
    rep = []
    def log(s=""):
        print(s); rep.append(str(s))

    log("nc_dir=%s date=%s grid=%dx%d area=%s" % (args.nc_dir, args.date, args.nx, args.ny, area))
    data = build(args.nc_dir, args.date, area, args.nx, args.ny)

    lat_order = args.lat_order
    if args.validate:
        log("\n=== 对拍 exampleData（lon-order=%s）===" % args.lon_order)
        score = {"asc": 0.0, "desc": 0.0}
        for kind in data:
            for field in data[kind]:
                try:
                    ex = load_example_field(args.example_dir, args.date, kind, field)
                except Exception as e:
                    log("  %s.%s 读 example 失败: %r" % (kind, field, e)); continue
                line = "  %-14s" % field
                for lo in ["desc", "asc"]:
                    mine = flatten(data[kind][field], lo, args.lon_order)
                    mean, mx, n = diff_flat(mine, ex)
                    score[lo] += mx
                    line += "  %s(mean=%.4g,max=%.4g,n=%d)" % (lo, mean, mx, n)
                log(line)
        lat_order = "desc" if score["desc"] <= score["asc"] else "asc"
        log("\n判定：lat-order = %s（desc 累计max=%.4g vs asc=%.4g；越小越吻合）" %
            (lat_order, score["desc"], score["asc"]))
    if lat_order == "auto":
        lat_order = "asc"   # 已由对拍 exampleData 定死：i=0 -> latMin(南)

    log("\n=== 生成 JSON（lat-order=%s, lon-order=%s）-> %s ===" % (lat_order, args.lon_order, args.out_dir))
    out_dir = os.path.join(args.out_dir, args.date) if not args.out_flat else args.out_dir
    for kind in data:
        fields_flat = {field: flatten(data[kind][field], lat_order, args.lon_order) for field in data[kind]}
        p = write_json(out_dir, args.date, area, args.nx, args.ny, kind, fields_flat)
        # 抽样报告
        anyf = next(iter(fields_flat))
        vals = [v for v in fields_flat[anyf] if v is not None]
        log("  %s -> %s  (%s: n=%d, min=%.4g, max=%.4g, null=%d)" %
            (kind, os.path.basename(p), anyf, len(fields_flat[anyf]),
             (min(vals) if vals else float('nan')), (max(vals) if vals else float('nan')),
             sum(1 for v in fields_flat[anyf] if v is None)))

    with open(os.path.join(BASE, "_preprocess_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(rep))
    print("\ndone -> _preprocess_report.txt")


if __name__ == "__main__":
    main()
