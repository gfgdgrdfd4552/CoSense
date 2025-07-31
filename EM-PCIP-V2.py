import numpy as np
import pandas as pd

# ========= 读取数据，与原代码相同 =========
C_raw = pd.read_csv(r'D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2info\illegal_parking_matrix.csv',      index_col=0).values.astype(int)
w_raw = pd.read_csv(r'D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2info\weighted_matrix.csv',             index_col=0).values.astype(float)
d_raw = pd.read_csv(r'D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2info\detection_confidence_matrix.csv', index_col=0).values.astype(float)


# ========= 可选：随机抽取 n 行（车辆） =========
# N_SELECT = 20     # 想抽多少行就改这里；None 或 >=总行数 表示使用全部
# SEED     = None  ###121     # 固定随机种子便于复现；设为 None 则每次不同
#
# if N_SELECT is not None and 0 < N_SELECT < C_raw.shape[0]:
#     rng = np.random.default_rng(SEED)
#     sel_idx = rng.choice(C_raw.shape[0], size=N_SELECT, replace=False)
#     # 若希望保持原始顺序以便对照，可排序；如果想保持随机顺序，把下面这行注释掉
#     sel_idx.sort()
#
#     C_raw = C_raw[sel_idx, :]
#     w_raw = w_raw[sel_idx, :]
#     d_raw = d_raw[sel_idx, :]
#
#     print(f"随机抽取 {N_SELECT} 行 (原始行索引): {sel_idx.tolist()}")
# else:
#     sel_idx = np.arange(C_raw.shape[0])
#     print("未抽样，使用全部行。")




#np.random.seed(121)
P_raw = 0.5 * w_raw + 0.5 * d_raw          # 置信度
S, M  = C_raw.shape


# 在 EM 循环之前或 compute_log_probs 一开始：
print("C_raw 中有没有 nan？", np.isnan(C_raw).any())
print("P_raw 中有没有 nan？", np.isnan(w_raw).any())
print("d_raw 中有没有 nan？", np.isnan(d_raw).any())


# 每一行的平均置信度（转换为百分比，保留两位小数）
row_means = np.round(P_raw.mean(axis=1) * 100, 2)

# 打印结果列表
print("每行平均置信度（百分数）:")
print(row_means.tolist())


# ========= 初始化 =========
P_raw = 1-P_raw
a, b      = 8,2
tol_Q     = 1e-5          # <-- Q 收敛阈值
max_iter  = 500

alpha = np.random.uniform(0.01, 0.99, size=S)
beta  = np.random.uniform(0.01, 0.99, size=S)
d     = 0.9

# ========= 工具函数 =========
def compute_log_probs(C, P, alpha, beta):
    eps = 1e-6
    a_c, b_c = np.clip(alpha, eps, 1-eps), np.clip(beta, eps, 1-eps)
    la, lna  = np.log(a_c),  np.log(1-a_c)
    lb, lnb  = np.log(b_c),  np.log(1-b_c)
    Cb = C.astype(bool)
    logP1 = (P * (C * la[:, None] + (~Cb) * lna[:, None])).sum(axis=0)
    logP0 = (P * (C * lnb[:, None] + (~Cb) * lb[:, None])).sum(axis=0)
    return logP1, logP0

def compute_Q(C, P, alpha, beta, d, mu, a, b):
    """期望对数后验 (常数项可忽略)，用于收敛判据。"""
    eps = 1e-12
    a_c, b_c = np.clip(alpha, eps, 1-eps), np.clip(beta, eps, 1-eps)

    # 先验 d 与 g_i
    Q_val  = np.sum(mu * np.log(np.clip(d, eps, 1-eps)) +
                    (1-mu) * np.log(np.clip(1-d, eps, 1-eps)))

    # Beta 先验 (常数 -log B(a,b) 可省略)
    Q_val += (a-1) * np.log(np.clip(d, eps, 1-eps)) + \
             (b-1) * np.log(np.clip(1-d, eps, 1-eps))

    # 观测似然
    Cb = C.astype(bool)
    log_alpha, log_1_a = np.log(a_c), np.log(1-a_c)
    log_beta,  log_1_b = np.log(b_c), np.log(1-b_c)

    # g_i = 1 部分
    Q_val += np.sum(mu   * P * ( C * log_alpha[:,None] + (~Cb) * log_1_a[:,None]))
    # g_i = 0 部分
    Q_val += np.sum((1-mu) * P * ( C * log_1_b[:,None] + (~Cb) * log_beta[:,None]))
    return Q_val

# ========= EM =========
Q_prev = None
for t in range(max_iter):
    # ---------- E step ----------
    logP1, logP0 = compute_log_probs(C_raw, P_raw, alpha, beta)
    e1 = np.exp(np.clip(logP1, -50, 50))
    e0 = np.exp(np.clip(logP0, -50, 50))
    mu = (d * e1) / (d * e1 + (1-d) * e0 + 1e-16)

    # ---------- M step ----------
    d = (a - 1 + mu.sum()) / (a + b - 2 + M)
    for j in range(S):
        pj, cj = P_raw[j], C_raw[j]
        alpha[j] = (pj * mu     *  cj).sum() / ((pj * mu    ).sum() + 1e-16)
        beta[j]  = (pj * (1-mu) * (1-cj)).sum() / ((pj*(1-mu)).sum() + 1e-16)

    alpha = np.clip(alpha, 1e-6, 1-1e-6)
    beta  = np.clip(beta,  1e-6, 1-1e-6)
    d     = np.clip(d,     1e-6, 1-1e-6)

    # ---------- Q 收敛判据 ----------
    Q_curr = compute_Q(C_raw, P_raw, alpha, beta, d, mu, a, b)
    if Q_prev is not None and abs(Q_curr - Q_prev) < tol_Q:
        print(f'Converged at iter {t},  |Q_delta|={abs(Q_curr-Q_prev):.2e}')
        break
    Q_prev = Q_curr
else:
    print('Reached max_iter without Q convergence.')

# ========= 硬标签输出 =========
G_hat = (mu > 0.5).astype(int)
print('G_hat:', G_hat)
print('alpha:', alpha)
print('beta :', beta)
print('d     :', d)
# ========= 输出每个目标车辆为违停的概率 =========
print("每个目标为违停的后验概率 mu（保留两位小数）:")
print(np.round(mu, 8).tolist())
