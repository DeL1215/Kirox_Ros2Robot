#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import numpy as np
from pathlib import Path
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split

# ---------------- 路徑設定（絕對路徑） ----------------
DATA_CSV  = Path("/home/jetson/ros2_ws/src/Kirox_Ros2Robot/kirox_robot/logreg/tri_scores.csv")
MODELS_DIR = Path("/home/jetson/ros2_ws/src/Kirox_Ros2Robot/kirox_robot/models")
MODELS_DIR.mkdir(parents=True, exist_ok=True)

# ---------------- 裝置 ----------------
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device: {device}")

# ---------------- 模型定義（小型 MLP） ----------------
class SimpleNN(nn.Module):
    def __init__(self):
        super(SimpleNN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Sigmoid()  # 二元分類機率輸出
        )
    def forward(self, x):
        return self.net(x)

# ---------------- 檢查資料存在 ----------------
if not DATA_CSV.exists():
    raise FileNotFoundError(f"找不到資料檔：{DATA_CSV}")

# ---------------- 讀取資料、處理缺值 ----------------
df = pd.read_csv(DATA_CSV)
df = df.fillna(0)  # NaN -> 0

X_np = df[["vision_score", "vad_score", "oww_score"]].values.astype(np.float32)
y_np = df[["output"]].values.astype(np.float32)

# ---------------- 訓練/驗證/測試切分 ----------------
# stratify 可維持 0/1 比例，增加穩定性
X_train, X_temp, y_train, y_temp = train_test_split(
    X_np, y_np, test_size=0.3, random_state=42, stratify=y_np
)
X_val, X_test, y_val, y_test = train_test_split(
    X_temp, y_temp, test_size=0.5, random_state=42, stratify=y_temp
)

# ---------------- 標準化（用訓練集均值/標準差） ----------------
mu = X_train.mean(axis=0)
sigma = X_train.std(axis=0)
sigma[sigma == 0] = 1.0  # 防零除

def standardize(x):
    return (x - mu) / sigma

X_train = standardize(X_train)
X_val   = standardize(X_val)
X_test  = standardize(X_test)

# 保存標準化參數
scaler_path = MODELS_DIR / "scaler.json"
with open(scaler_path, "w") as f:
    json.dump({"mu": mu.tolist(), "sigma": sigma.tolist()}, f)

# ---------------- 轉 Tensor 並搬到裝置 ----------------
X_train_t = torch.tensor(X_train, dtype=torch.float32, device=device)
y_train_t = torch.tensor(y_train, dtype=torch.float32, device=device)
X_val_t   = torch.tensor(X_val,   dtype=torch.float32, device=device)
y_val_t   = torch.tensor(y_val,   dtype=torch.float32, device=device)
X_test_t  = torch.tensor(X_test,  dtype=torch.float32, device=device)
y_test_t  = torch.tensor(y_test,  dtype=torch.float32, device=device)

# ---------------- 類別不平衡加權的 BCE ----------------
pos_ratio = float(y_train_t.mean().item())  # 正類比例
neg_ratio = 1.0 - pos_ratio
w_pos = 0.5 / max(pos_ratio, 1e-6)  # 正類越少 -> 權重越大
w_neg = 0.5 / max(neg_ratio, 1e-6)

def bce_weighted(pred, target):
    # pred, target shape: (N,1)，pred 已是 Sigmoid 機率
    weights = torch.where(target > 0.5,
                          torch.tensor(w_pos, device=device),
                          torch.tensor(w_neg, device=device))
    bce = -(target * torch.log(pred.clamp_min(1e-7)) +
            (1 - target) * torch.log((1 - pred).clamp_min(1e-7)))
    return (weights * bce).mean()

def accuracy_at_threshold(prob, y_true, thr):
    pred = (prob >= thr).float()
    return (pred.eq(y_true).sum().item() / y_true.size(0)) * 100.0

# ---------------- 建立模型、優化器、Early Stopping ----------------
model = SimpleNN().to(device)
optimizer = optim.Adam(model.parameters(), lr=5e-3, weight_decay=1e-4)  # L2 正則
best_val_loss = float("inf")
patience = 400
no_improve = 0

# ---------------- 訓練 ----------------
max_epochs = 5000
for epoch in range(1, max_epochs + 1):
    model.train()
    optimizer.zero_grad()
    out_train = model(X_train_t)
    loss = bce_weighted(out_train, y_train_t)
    loss.backward()
    optimizer.step()

    if epoch % 100 == 0:
        model.eval()
        with torch.no_grad():
            prob_train = model(X_train_t)
            acc_train = accuracy_at_threshold(prob_train, y_train_t, 0.5)

            prob_val = model(X_val_t)
            val_loss = bce_weighted(prob_val, y_val_t).item()
            acc_val = accuracy_at_threshold(prob_val, y_val_t, 0.5)

        print(f"Epoch {epoch}, Loss: {loss.item():.4f}, "
              f"Train Acc: {acc_train:.2f}%, Val Loss: {val_loss:.4f}, Val Acc: {acc_val:.2f}%")

        # Early stopping 依照 Val Loss
        if val_loss + 1e-6 < best_val_loss:
            best_val_loss = val_loss
            no_improve = 0
            torch.save(model.state_dict(), MODELS_DIR / "logreg_model_v2.best.pth")
        else:
            no_improve += 100
            if no_improve >= patience:
                print(f"早停觸發於 Epoch {epoch}（最佳 Val Loss: {best_val_loss:.4f}）")
                break

# ---------------- 載入最佳驗證表現的模型 ----------------
best_model_path = MODELS_DIR / "logreg_model_v2.best.pth"
if best_model_path.exists():
    model.load_state_dict(torch.load(best_model_path, map_location=device))
else:
    # 若沒有最佳檔（例如資料很穩定），則存最後權重
    torch.save(model.state_dict(), best_model_path)

# ---------------- 在驗證集上最佳化閾值（Youden J） ----------------
model.eval()
with torch.no_grad():
    prob_val = model(X_val_t).detach().cpu().numpy().ravel()
    y_val_np = y_val_t.detach().cpu().numpy().ravel()

def best_threshold_by_youden(prob, y_true):
    ths = np.linspace(0.05, 0.95, 19)
    best_thr, best_score = 0.5, -1
    for t in ths:
        pred = (prob >= t).astype(np.float32)
        tp = np.sum((pred == 1) & (y_true == 1))
        tn = np.sum((pred == 0) & (y_true == 0))
        fp = np.sum((pred == 1) & (y_true == 0))
        fn = np.sum((pred == 0) & (y_true == 1))
        tpr = tp / (tp + fn + 1e-9)
        fpr = fp / (fp + tn + 1e-9)
        score = tpr - fpr
        if score > best_score:
            best_score = score
            best_thr = t
    return best_thr, best_score

best_thr, best_score = best_threshold_by_youden(prob_val, y_val_np)
print(f"✅ 最佳化閾值(Youden J): {best_thr:.2f}（分數 {best_score:.3f}）")

# ---------------- 測試集評估（使用最佳閾值） ----------------
with torch.no_grad():
    prob_test = model(X_test_t)
    acc_test = accuracy_at_threshold(prob_test, y_test_t, best_thr)
print(f"✅ 測試集準確率(最佳閾值 {best_thr:.2f}): {acc_test:.2f}%")

# ---------------- 保存最終模型與設定 ----------------
final_model_path = MODELS_DIR / "logreg_model_v2.pth"
torch.save(model.state_dict(), final_model_path)
print(f"✅ 模型已保存 -> {final_model_path}")
print(f"✅ 標準化參數 -> {scaler_path}")

# 也把最佳閾值與裝置資訊寫入 config，方便推論端使用
config_path = MODELS_DIR / "config.json"
with open(config_path, "w") as f:
    json.dump({"best_threshold": float(best_thr),
               "device": str(device)}, f, ensure_ascii=False, indent=2)
print(f"✅ 設定已保存 -> {config_path}")

# ---------------- 推理解耦函式（供其他模組 import 使用） ----------------
def load_scaler(path: Path):
    with open(path, "r") as f:
        s = json.load(f)
    mu = np.array(s["mu"], dtype=np.float32)
    sigma = np.array(s["sigma"], dtype=np.float32)
    sigma[sigma == 0] = 1.0
    return mu, sigma

def load_infer(best_model=final_model_path, scaler_file=scaler_path, threshold_file=config_path):
    m = SimpleNN().to(device)
    m.load_state_dict(torch.load(best_model, map_location=device))
    m.eval()
    mu_, sigma_ = load_scaler(scaler_file)
    thr = json.load(open(threshold_file, "r"))["best_threshold"]
    return m, mu_, sigma_, float(thr)

def predict_triplet(tri, model_obj=None, mu_=None, sigma_=None, thr: float = None):
    """
    tri: list/tuple [vision_score, vad_score, oww_score]
    回傳: (probability, predicted_label)
    """
    if model_obj is None or mu_ is None or sigma_ is None or thr is None:
        model_obj, mu_, sigma_, thr = load_infer()

    tri = np.array(tri, dtype=np.float32)
    tri_std = (tri - mu_) / sigma_
    x = torch.tensor(tri_std[None, :], dtype=torch.float32, device=device)
    with torch.no_grad():
        p = model_obj(x).item()
        y = 1 if p >= thr else 0
    return p, y

# ---------------- 推理示例 ----------------
if __name__ == "__main__":
    p, y = predict_triplet([0.004219, 0.572762, 0.992587])
    print(f"Probability: {p:.3f}, Predicted(@thr={best_thr:.2f}): {y}")
