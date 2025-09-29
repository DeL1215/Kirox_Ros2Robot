import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd

# --- 模型定義 ---
class LogisticRegression(nn.Module):
    def __init__(self):
        super(LogisticRegression, self).__init__()
        self.fc = nn.Linear(3, 1)    # 3個輸入 -> 1個輸出
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        x = self.fc(x)
        x = self.sigmoid(x)  # 輸出機率
        return x

# --- 讀取 CSV ---
df = pd.read_csv("/home/del1215/ros2_ws/src/kirox_robot/kirox_robot/logreg/tri_scores.csv")  # 這裡放你的 CSV 檔案名稱

# 特徵欄位對應
X = torch.tensor(df[["vision_score", "vad_score", "oww_score"]].values, dtype=torch.float32)
y = torch.tensor(df[["output"]].values, dtype=torch.float32)

# --- 建立模型 ---
model = LogisticRegression()
criterion = nn.BCELoss()              # 二元交叉熵
optimizer = optim.Adam(model.parameters(), lr=0.01)

# --- 訓練 ---
for epoch in range(10000):
    optimizer.zero_grad()
    outputs = model(X)
    loss = criterion(outputs, y)
    loss.backward()
    optimizer.step()
    if (epoch+1) % 200 == 0:
        print(f"Epoch {epoch+1}, Loss: {loss.item():.4f}")

# --- 保存模型 ---
torch.save(model.state_dict(), "logreg_model.pth")
print("✅ 模型已保存 -> logreg_model.pth")

# --- 載入模型 ---
loaded_model = LogisticRegression()
loaded_model.load_state_dict(torch.load("logreg_model.pth"))
loaded_model.eval()
print("✅ 模型已載入")

# --- 推理 (舉例一筆資料) ---
with torch.no_grad():
    test_input = torch.tensor([[0.004219, 0.572762, 0.992587]], dtype=torch.float32)  # 你可以換掉數字
    prob = loaded_model(test_input).item()        # 機率
    pred = torch.round(loaded_model(test_input)).item()  # 四捨五入成 0/1
    print(f"Probability: {prob:.3f}, Predicted: {int(pred)}")
