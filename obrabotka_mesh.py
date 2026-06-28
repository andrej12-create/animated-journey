import math
import os
import tkinter as tk
from tkinter import filedialog
from collections import defaultdict

TABLE_CENTER_MM = 110.0
Z_STEP_MM = 1.0
R_MIN = 5.0
R_MAX = 105.0
N_POINTS = 200

#  Фильтры
def iqr_filter(angles, radii):
    if len(radii) < 4:
        return angles, radii
    s = sorted(radii)
    n = len(s)
    q1 = s[n // 4]
    q3 = s[3 * n // 4]
    iqr = q3 - q1
    low = q1 - 1.0 * iqr
    high = q3 + 1.0 * iqr
    res = [(a, r) for a, r in zip(angles, radii) if low <= r <= high]
    if len(res) < 10:
        return angles, radii
    return [x[0] for x in res], [x[1] for x in res]

def neighbor_outlier_filter(angles, radii, threshold=5.0, window=5):
    n = len(radii)
    if n < window + 1:
        return angles, radii
    res_a, res_r = [], []
    half = window // 2
    for i in range(n):
        neigh = []
        for j in range(-half, half + 1):
            idx = (i + j) % n
            if idx != i:
                neigh.append(radii[idx])
        avg = sum(neigh) / len(neigh)
        if abs(radii[i] - avg) <= threshold:
            res_a.append(angles[i])
            res_r.append(radii[i])
    if len(res_r) < 10:
        return angles, radii
    return res_a, res_r

def bilateral_smooth(angles, radii, window=7, sigma_r=4.0):
    n = len(radii)
    if n < window:
        return radii
    result = []
    half = window // 2
    for i in range(n):
        wsum = 0.0
        rsum = 0.0
        for j in range(max(0, i - half), min(n, i + half + 1)):
            da = abs(angles[i] - angles[j])
            da = min(da, 360.0 - da)
            w_space = math.exp(-(da ** 2) / (2.0 * ((window / 2.0) ** 2)))
            dr = abs(radii[i] - radii[j])
            w_range = math.exp(-(dr ** 2) / (2.0 * (sigma_r ** 2)))
            w = w_space * w_range
            wsum += w
            rsum += radii[j] * w
        result.append(rsum / wsum if wsum > 0 else radii[i])
    return result

def remove_isolated(cart, threshold=3.5):
    if not cart:
        return []
    keep = []
    for i, (x1, y1) in enumerate(cart):
        for j, (x2, y2) in enumerate(cart):
            if i != j and math.hypot(x1 - x2, y1 - y2) < threshold:
                keep.append((x1, y1))
                break
    return keep

#  Векторные операции
def vec_sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def vec_cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def vec_len(v):
    return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def vec_norm(v):
    l = vec_len(v)
    if l == 0:
        return (0,0,1)
    return (v[0]/l, v[1]/l, v[2]/l)

def vec_add(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

#  Основная обработка 
def process_scan_to_obj(input_csv, output_obj):
    raw_layers = defaultdict(list)

    with open(input_csv, 'r', encoding='utf-8') as f:
        next(f)
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != 3:
                continue
            angle_deg = float(parts[0])
            layer = int(float(parts[1]))
            dist = float(parts[2])
            if dist <= 0 or dist >= TABLE_CENTER_MM:
                continue
            r = TABLE_CENTER_MM - dist
            if r < R_MIN or r > R_MAX:
                continue
            raw_layers[layer].append((angle_deg, r, layer * Z_STEP_MM))

    resampled_layers = []

    for layer in sorted(raw_layers.keys()):
        pts = raw_layers[layer]
        if len(pts) < 10:
            continue
        pts.sort(key=lambda x: x[0])
        angles = [p[0] for p in pts]
        radii = [p[1] for p in pts]
        z = pts[0][2]

        angles_f, radii_f = iqr_filter(angles, radii)
        angles_f, radii_f = neighbor_outlier_filter(angles_f, radii_f, threshold=5.0, window=5)
        if len(radii_f) < 10:
            continue

        radii_s = bilateral_smooth(angles_f, radii_f, window=7, sigma_r=4.0)

        cart = []
        for a, r_sm in zip(angles_f, radii_s):
            ar = math.radians(a)
            x = r_sm * math.cos(ar)
            y = -r_sm * math.sin(ar)
            cart.append((x, y))

        cart = remove_isolated(cart, threshold=3.5)
        if len(cart) < 10:
            continue

        cx = sum(p[0] for p in cart) / len(cart)
        cy = sum(p[1] for p in cart) / len(cart)
        centered = [(x - cx, y - cy) for x, y in cart]

        polar = []
        for x, y in centered:
            r = math.hypot(x, y)
            a = math.degrees(math.atan2(-y, x))
            if a < 0:
                a += 360.0
            polar.append((a, r))
        polar.sort(key=lambda x: x[0])

        resampled = []
        if len(polar) >= 2:
            pa = [p[0] for p in polar] + [polar[0][0] + 360.0]
            pr = [p[1] for p in polar] + [polar[0][1]]
            for i in range(N_POINTS):
                target = i * (360.0 / N_POINTS)
                for k in range(len(pa) - 1):
                    if pa[k] <= target <= pa[k + 1]:
                        t = (target - pa[k]) / (pa[k + 1] - pa[k])
                        rr = pr[k] + t * (pr[k + 1] - pr[k])
                        ar = math.radians(target)
                        x = rr * math.cos(ar)
                        y = -rr * math.sin(ar)
                        resampled.append((x, y, z))
                        break
                else:
                    rr = polar[-1][1]
                    ar = math.radians(target)
                    x = rr * math.cos(ar)
                    y = -rr * math.sin(ar)
                    resampled.append((x, y, z))
        else:
            continue

        if len(resampled) == N_POINTS:
            resampled_layers.append(resampled)

    if len(resampled_layers) < 2:
        print("Слишком мало слоёв для построения поверхности!")
        return None

    n_layers = len(resampled_layers)
    z_top = resampled_layers[-1][0][2]

    # Собираем вершины
    verts = []
    for layer_pts in resampled_layers:
        for x, y, z in layer_pts:
            verts.append((x, y, z))
    cap_center_idx = len(verts)  # 0-based, потом +1 для OBJ
    verts.append((0.0, 0.0, z_top))

    # Собираем грани (треугольники) и вычисляем нормали
    faces = []  # список (v1, v2, v3) 0-based
    # Боковые грани
    for i in range(n_layers - 1):
        for j in range(N_POINTS):
            curr = i * N_POINTS + j
            next_j = (j + 1) % N_POINTS
            curr_next = i * N_POINTS + next_j
            below = (i + 1) * N_POINTS + j
            below_next = (i + 1) * N_POINTS + next_j
            faces.append((curr, curr_next, below))
            faces.append((curr_next, below_next, below))
    # Крышка
    top_start = (n_layers - 1) * N_POINTS
    for j in range(N_POINTS):
        a = top_start + j
        b = top_start + (j + 1) % N_POINTS
        faces.append((cap_center_idx, a, b))

    # Вычисляем нормали вершин
    vnorms = [(0.0, 0.0, 0.0) for _ in range(len(verts))]
    for f in faces:
        v1, v2, v3 = f
        p1 = verts[v1]
        p2 = verts[v2]
        p3 = verts[v3]
        n = vec_norm(vec_cross(vec_sub(p2, p1), vec_sub(p3, p1)))
        vnorms[v1] = vec_add(vnorms[v1], n)
        vnorms[v2] = vec_add(vnorms[v2], n)
        vnorms[v3] = vec_add(vnorms[v3], n)

    for i in range(len(vnorms)):
        vnorms[i] = vec_norm(vnorms[i])

    # Пишем OBJ
    obj_lines = []
    obj_lines.append("# 3D Scan Mesh with normals and top cap")
    obj_lines.append(f"# Layers: {n_layers}, Points per layer: {N_POINTS}")
    for v in verts:
        obj_lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for vn in vnorms:
        obj_lines.append(f"vn {vn[0]:.4f} {vn[1]:.4f} {vn[2]:.4f}")
    for f in faces:
        # OBJ индексы 1-based, v//vn одинаковые индексы
        obj_lines.append(f"f {f[0]+1}//{f[0]+1} {f[1]+1}//{f[1]+1} {f[2]+1}//{f[2]+1}")

    obj_content = "\n".join(obj_lines)
    with open(output_obj, 'w', encoding='utf-8') as f:
        f.write(obj_content)

    total_verts = len(verts)
    total_faces = len(faces)
    print(f"\nOBJ: Вершин {total_verts}, Граней {total_faces}")
    print(f"Сохранён: {output_obj}")
    return obj_content

#  GUI 
def main():
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)

    # 1. Выбор входного файла
    input_csv = filedialog.askopenfilename(
        title="Выберите файл скана (CSV)",
        filetypes=[("CSV files", "*.CSV *.csv"), ("All files", "*.*")]
    )
    if not input_csv:
        print("Отменено.")
        return

    # 2. Выбор папки для сохранения (имя файла будет автоматическим)
    save_dir = filedialog.askdirectory(title="Выберите папку для сохранения OBJ")
    if not save_dir:
        print("Сохранение отменено.")
        return
    
    base_name = os.path.splitext(os.path.basename(input_csv))[0]
    output_obj = os.path.join(save_dir, base_name + "_mesh.obj")

    print(f"Обработка: {os.path.basename(input_csv)} ...")
    print(f"Сохранение в: {output_obj}")
    process_scan_to_obj(input_csv, output_obj)

if __name__ == "__main__":
    main()