import os
import matplotlib.pyplot as plt
import pandas as pd

# 💡 日本語文字化け対策（Windows標準フォントの指定）
plt.rcParams["font.family"] = "MS Gothic"

file_wave1 = "sim( 40.0 _ 3 _ 6 _ 1.0).csv"
file_wave2 = "sim( 25.0 _ 3 _ 6 _ 1.0).csv"

if not (os.path.exists(file_wave1) and os.path.exists(file_wave2)):
  print(
      "❌ CSVファイルが見つかりません。2パターンのシミュレーションを完了させてください。"
  )
  exit()

# CSV読み込み
df1 = pd.read_csv(file_wave1)
df2 = pd.read_csv(file_wave2)

# グラフの設定
plt.figure(figsize=(10, 5))

# 横軸：時間(time)、縦軸：速度(vel_x_cms)
plt.plot(
    df1["time"],
    df1["vel_x_cms"],
    label=f"{file_wave1}",
    color="blue",
    linewidth=1.5,
)
plt.plot(
    df2["time"],
    df2["vel_x_cms"],
    label=f"{file_wave2}",
    color="red",
    linewidth=1.5,
)

# グラフ装飾
plt.title(
    "振れ幅:25.0[deg]  上下運動の周波数のみ比較  ヒレの数:6 形成波:1",
    fontsize=14,
    fontweight="bold",
)
plt.xlabel("Time [s]", fontsize=12)
plt.ylabel("Forward Velocity [cm/s]", fontsize=12)
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend(fontsize=11)

# 画像保存と表示
output_img = "velocity_comparison.png"
plt.savefig(output_img, dpi=300, bbox_inches="tight")
print(f"📊 グラフを保存しました: {output_img}")
plt.show()