import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

# 1. 设置全局字体风格（模仿学术论文）
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams['font.size'] = 11
plt.rcParams['axes.linewidth'] = 1.0  # 边框粗细

def interpolate_path_uniform(data, target_num_points):
    """
    将路径数据插值为等间距的点，数量为target_num_points
    
    参数:
        data: Nx4 数组 (x, y, yaw, distance)
        target_num_points: 目标点数
    
    返回:
        interpolated_data: 插值后的Nx4数组
    """
    if data.shape[0] < 2:
        return data
    
    x, y, yaw, dist = data.T
    
    # 计算原始路径的累积弧长
    dx = np.diff(x)
    dy = np.diff(y)
    segment_lengths = np.sqrt(dx**2 + dy**2)
    cumulative_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    
    # 去除重复点（累积弧长相同的点）
    # 保留第一个点，然后只保留与前一个点弧长不同的点
    tolerance = 1e-10
    unique_mask = np.concatenate([[True], np.diff(cumulative_length) > tolerance])
    
    cumulative_length = cumulative_length[unique_mask]
    x = x[unique_mask]
    y = y[unique_mask]
    yaw = yaw[unique_mask]
    dist = dist[unique_mask]
    
    print(f"  去重后保留 {len(x)} 个有效点")
    
    # 检查是否有足够的点进行插值
    if len(x) < 2:
        print("  警告: 去重后点数不足，返回原始数据")
        return data
    
    # 创建等间距的弧长采样点
    total_length = cumulative_length[-1]
    uniform_length = np.linspace(0, total_length, target_num_points)
    
    # 根据有效点数选择插值方法
    if len(x) < 4:
        interp_kind = 'linear'
        print(f"  使用线性插值（点数少于4）")
    else:
        interp_kind = 'cubic'
        print(f"  使用三次样条插值")
    
    # 对每个维度进行插值
    interp_x = interp1d(cumulative_length, x, kind=interp_kind, fill_value='extrapolate')
    interp_y = interp1d(cumulative_length, y, kind=interp_kind, fill_value='extrapolate')
    interp_yaw = interp1d(cumulative_length, yaw, kind='linear', fill_value='extrapolate')
    interp_dist = interp1d(cumulative_length, dist, kind='linear', fill_value='extrapolate')
    
    # 生成插值后的数据
    x_new = interp_x(uniform_length)
    y_new = interp_y(uniform_length)
    yaw_new = interp_yaw(uniform_length)
    dist_new = interp_dist(uniform_length)
    
    interpolated_data = np.column_stack([x_new, y_new, yaw_new, dist_new])
    
    return interpolated_data

def calculate_derivatives_and_integral(data, dt):
    """
    使用numpy.gradient计算曲率和累积平方加速度。
    """
    if data.shape[0] < 3:
        return {}, {}

    N = data.shape[0]
    x, y, _, dist_obs = data.T

    # --- 链式计算导数 ---
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    ax = np.gradient(vx, dt)
    ay = np.gradient(vy, dt)

    # --- 计算关键指标 ---
    # 1. 累积平方加速度 J(t) = integral(|a(tau)|^2) d(tau)
    a_mag_sq = ax**2 + ay**2
    integrated_acc_sq = np.cumsum(a_mag_sq) * dt

    # 2. 曲率 k
    v = np.sqrt(vx**2 + vy**2)
    v_for_div = np.copy(v)
    v_for_div[v_for_div < 1e-6] = 1e-6
    numerator = vx * ay - vy * ax
    denominator = v_for_div**3
    k = numerator / denominator

    # 时间轴
    t = np.arange(N) * dt

    calcs = {'integrated_acc': integrated_acc_sq, 'k': k, 'd': dist_obs}
    times = {'t': t}

    return calcs, times

# --- 2. 数据加载和预处理 ---
dt = 0.2  # 原始假设的路径点之间的时间间隔

try:
    baseline_data = np.loadtxt('path.txt')
    print(f"成功加载 path.txt: {baseline_data.shape[0]} 个点")
except IOError:
    print("警告: 未找到 'path.txt' 文件.")
    baseline_data = np.array([])

try:
    ours_data_raw = np.loadtxt('quintic_path.txt')
    print(f"成功加载 quintic_path.txt: {ours_data_raw.shape[0]} 个点")
except IOError:
    print("警告: 未找到 'quintic_path.txt' 文件.")
    ours_data_raw = np.array([])

# 预处理：将quintic_path插值为与path相同数量的等间距点
if baseline_data.size > 0 and ours_data_raw.size > 0:
    target_num_points = baseline_data.shape[0]
    
    print(f"\n开始预处理：将 quintic_path 插值为 {target_num_points} 个等间距点...")
    ours_data = interpolate_path_uniform(ours_data_raw, target_num_points)
    print(f"插值完成: {ours_data.shape[0]} 个点")
    
    # 验证数据长度一致
    assert baseline_data.shape[0] == ours_data.shape[0], "数据长度不一致!"
    print(f"数据已统一为 {baseline_data.shape[0]} 个点.\n")
else:
    ours_data = ours_data_raw

# 分别计算 (使用完整的, 统一长度的数据)
calcs_base, times_base = calculate_derivatives_and_integral(baseline_data, dt)
calcs_ours, times_ours = calculate_derivatives_and_integral(ours_data, dt)

# ===== 可视化范围配置 =====
# 设置要跳过的开头和结尾点数
SKIP_START = 0  # 跳过开头的点数（例如：5 表示跳过前5个点）
SKIP_END = 10    # 跳过结尾的点数（例如：10 表示跳过后10个点）

# 应用可视化范围裁剪
if SKIP_START > 0 or SKIP_END > 0:
    end_idx = -SKIP_END if SKIP_END > 0 else None
    
    # 裁剪baseline数据
    if 't' in times_base and times_base['t'].size > 0:
        times_base['t'] = times_base['t'][SKIP_START:end_idx]
        for key in calcs_base:
            calcs_base[key] = calcs_base[key][SKIP_START:end_idx]
    
    # 裁剪ours数据
    if 't' in times_ours and times_ours['t'].size > 0:
        times_ours['t'] = times_ours['t'][SKIP_START:end_idx]
        for key in calcs_ours:
            calcs_ours[key] = calcs_ours[key][SKIP_START:end_idx]
    
    print(f"可视化范围: 跳过开头 {SKIP_START} 个点, 跳过结尾 {SKIP_END} 个点")
# ===========================

# --- 3. 开始绘图 ---
# 更新布局为 1x3
fig, axs = plt.subplots(1, 3, figsize=(18, 5))
plt.subplots_adjust(wspace=0.25)

# 定义采样间隔
sample_interval = 5

# --- 对计算结果进行采样 (每隔五个点显示) ---
if 't' in times_base:
    times_base['t'] = times_base['t'][::sample_interval]
    for key in calcs_base:
        calcs_base[key] = calcs_base[key][::sample_interval]

if 't' in times_ours:
    times_ours['t'] = times_ours['t'][::sample_interval]
    for key in calcs_ours:
        calcs_ours[key] = calcs_ours[key][::sample_interval]


# 定义统一的样式
line_width = 2.0
col_ours = '#2ca02c'  # 绿色
col_base = '#d62728'  # 红色
limit_style = {'color': 'black', 'linestyle': '--', 'linewidth': 1.5, 'alpha': 0.7}

# 获取绘图的时间范围
t_max = 0
if 't' in times_base and times_base['t'].size > 0:
    t_max = max(t_max, times_base['t'][-1])
if 't' in times_ours and times_ours['t'].size > 0:
    t_max = max(t_max, times_ours['t'][-1])
if t_max == 0:
    t_max = 25  # 默认值

# --- 子图 1: 累积平方加速度 ---
ax = axs[0]
if 'integrated_acc' in calcs_base:
    ax.plot(times_base['t'], calcs_base['integrated_acc'], color=col_base, label='Baseline', linewidth=line_width, alpha=0.8)
if 'integrated_acc' in calcs_ours:
    ax.plot(times_ours['t'], calcs_ours['integrated_acc'], color=col_ours, label='Ours (Proposed)', linewidth=line_width)
ax.set_title('(a) Integrated Squared Acceleration', fontsize=13, pad=10)
ax.set_ylabel(r'$J(t) = \int_{0}^{t} |a(\tau)|^2 d\tau$', fontsize=12)
ax.set_xlabel(r'$t \, (s)$', fontsize=12)
ax.set_xlim(0, t_max)
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='upper left', frameon=True, fontsize=10)

# --- 子图 2: 曲率 ---
ax = axs[1]
if 'k' in calcs_base:
    ax.plot(times_base['t'], calcs_base['k'], color=col_base, linewidth=line_width, alpha=0.8)
if 'k' in calcs_ours:
    ax.plot(times_ours['t'], calcs_ours['k'], color=col_ours, linewidth=line_width)
k_limit = 0.2
ax.hlines([k_limit, -k_limit], 0, t_max, **limit_style)
ax.set_title('(b) Curvature Profile', fontsize=13, pad=10)
ax.set_ylabel(r'$\kappa \, (m^{-1})$', fontsize=12)
ax.set_xlabel(r'$t \, (s)$', fontsize=12)
ax.set_xlim(0, t_max)
ax.set_ylim(-0.4, 0.4)
ax.grid(True, linestyle=':', alpha=0.6)
ax.text(t_max * 0.6, 0.22, r'$\kappa_{max}$', fontsize=11)

# --- 子图 3: 障碍物距离 ---
ax = axs[2]
if 'd' in calcs_base:
    ax.plot(times_base['t'], calcs_base['d'], color=col_base, linewidth=line_width, alpha=0.8)
if 'd' in calcs_ours:
    ax.plot(times_ours['t'], calcs_ours['d'], color=col_ours, linewidth=line_width)
d_safe = 0.5
ax.hlines(d_safe, 0, t_max, **limit_style)
ax.set_title('(d) Distance to Obstacle', fontsize=13, pad=10)
ax.set_ylabel(r'$d_{obs} \, (m)$', fontsize=12)
ax.set_xlabel(r'$t \, (s)$', fontsize=12)
ax.set_xlim(0, t_max)
# 动态调整Y轴范围
d_max_val = 0
if 'd' in calcs_base and calcs_base['d'].size > 0:
    d_max_val = max(d_max_val, np.max(calcs_base['d']))
if 'd' in calcs_ours and calcs_ours['d'].size > 0:
    d_max_val = max(d_max_val, np.max(calcs_ours['d']))
ax.set_ylim(0, d_max_val * 1.1 if d_max_val > 0 else 2.5)
ax.grid(True, linestyle=':', alpha=0.6)
ax.text(t_max * 0.1, 0.6, 'Safety Margin', fontsize=10)

# 保存和显示
plt.tight_layout()
# plt.savefig('trajectory_comparison.png', dpi=300)
plt.show()