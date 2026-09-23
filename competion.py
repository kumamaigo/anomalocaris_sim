import os
import matplotlib.pyplot as plt
import pandas as pd

file_wave1 = "sim_result_wave_1.0.csv"
file_wave2 = "sim_result_wave_2.0.csv"

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
    label="Wave = 1.0 (1波長)",
    color="blue",
    linewidth=1.5,
)
plt.plot(
    df2["time"],
    df2["vel_x_cms"],
    label="Wave = 2.0 (2波長)",
    color="red",
    linewidth=1.5,
)

# グラフ装飾
plt.title(
    "Anomalocaris Swimming Velocity Comparison (6 Fins)",
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