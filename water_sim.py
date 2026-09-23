import csv  # シミュレーション結果をCSVファイルに保存するためのライブラリ
import math  # 円周率(pi)や三角関数(sin)、角度変換(radians)に使うライブラリ
import sys  # システム終了処理（sys.exit）などに使うライブラリ
import time  # 実行速度の制御（スリープ）や実時間計測に使うライブラリ
import mujoco  # 物理シミュレーションエンジン「MuJoCo」の本体
import mujoco.viewer  # MuJoCoの3D描画（ウインドウ表示）用モジュール

# ==========================================
# 📍 動作・流体パラメータ
# ==========================================
AMPLITUDE_DEG = 25.0  # ヒレの片振幅（最大傾斜角度 = 25度）
FREQ = 1.5  # ヒレの上下動の周波数（1秒間に1.5周期）
SIM_SPEED = 1.0  # シミュレーションの再生速度倍率（1.0 = 実時間と同じスピード）

NUM_FINS = 6  # 片側のヒレの枚数（6枚）
NUMBER_OF_WAVES = 2.0  # 体全体で形成する波（波動）の数（1.0なら1波長、2.0なら2波長）
PHASE_SHIFT = (
    2 * math.pi * NUMBER_OF_WAVES
) / (
    NUM_FINS - 1
)  # 隣り合うヒレ同士の位相差（ラジアン）を計算

X_START = 0.110  # 最も前方のヒレ（ヒレ0）のX軸方向の位置 [m]
X_STEP = 0.055  # ヒレとヒレの間隔（X軸方向） [m]
Y_LEFT = 0.030  # 左ヒレの付け根のY軸位置 [m]
Y_RIGHT = -0.030  # 右ヒレの付け根のY軸位置 [m]
Z_BODY = 1.0  # 胴体の初期高さ（Z軸位置） [m]
FIN_LENGTH = 0.12  # ヒレの長さ [m]
# ==========================================

is_paused = False  # 一時停止フラグ（False: 実行中, True: 一時停止中）


# キーボード入力時に実行される関数（スペースキーで一時停止/再開）
def key_callback(keycode):
  global is_paused  # グローバル変数の is_paused を書き換える宣言
  if keycode == 32:  # キーコード 32（スペースキー）が押された場合
    is_paused = not is_paused  # 一時停止状態を反転（切り替え）


# MuJoCoに読み込ませるロボットのモデル構造（XMLテキスト）を自動生成する関数
def generate_xml():
  left_fins_xml = ""  # 左ヒレ群のXML構造を保持する文字列
  right_fins_xml = ""  # 右ヒレ群のXML構造を保持する文字列
  actuators_xml = ""  # ヒレを動かすモーター（アクチュエータ）のXML文字列
  skin_vertices_l, skin_vertices_r = (
      "",
      "",
  )  # 左右のヒレ膜（スキン）の頂点座標データ
  skin_bones_l, skin_bones_r = (
      "",
      "",
  )  # ヒレ膜と関節（ボーン）の連動設定データ
  skin_faces_l, skin_faces_r = (
      "",
      "",
  )  # ヒレ膜を生成する三角形ポリゴンの面データ

  # ヒレの枚数（NUM_FINS）分だけループしてXMLを組み立てる
  for i in range(NUM_FINS):
    x_pos = X_START - (i * X_STEP)  # i番目のヒレのX座標を後方へずらして計算

    # 左側i番目のヒレのパーツ（Body）、関節（Joint）、メッシュ（Geom）を定義
    left_fins_xml += f"""
      <body name="fin_l{i}" pos="{x_pos:.4f} {Y_LEFT:.4f} 0">
        <joint name="joint_l{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" rgba="0.8 0.3 0.2 1" contype="0" conaffinity="0"/>
      </body>"""

    # 右側i番目のヒレのパーツ（Body）、関節（Joint）、メッシュ（Geom）を定義
    right_fins_xml += f"""
      <body name="fin_r{i}" pos="{x_pos:.4f} {Y_RIGHT:.4f} 0">
        <joint name="joint_r{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" euler="0 0 180" rgba="0.2 0.8 0.3 1" contype="0" conaffinity="0"/>
      </body>"""

    # 左右の関節を駆動させる位置制御モータ（アクチュエータ）を追加
    actuators_xml += f"""
    <position joint="joint_l{i}" kp="10"/>
    <position joint="joint_r{i}" kp="10"/>"""

    # 左ヒレの「根元」と「先端」の頂点座標文字列を作成
    v_base_l = f" {x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}"
    v_tip_l = f" {x_pos:.4f} {Y_LEFT + FIN_LENGTH:.4f} {Z_BODY:.4f}"
    skin_vertices_l += v_base_l + v_tip_l  # 頂点リストに追加
    # 左ヒレの骨組みと頂点を対応付け（関節の動きに合わせて膜を伸縮させる設定）
    skin_bones_l += f"""
      <bone body="fin_l{i}" bindpos="{x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

    # 右ヒレの「根元」と「先端」の頂点座標文字列を作成
    v_base_r = f" {x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}"
    v_tip_r = f" {x_pos:.4f} {Y_RIGHT - FIN_LENGTH:.4f} {Z_BODY:.4f}"
    skin_vertices_r += v_base_r + v_tip_r  # 頂点リストに追加
    # 右ヒレの骨組みと頂点を対応付け
    skin_bones_r += f"""
      <bone body="fin_r{i}" bindpos="{x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

    # 隣り合うヒレ同士の間に膜を張るため、三角形のメッシュ面を生成する設定
    if i < NUM_FINS - 1:
      b1, t1 = i * 2, i * 2 + 1  # 前のヒレの根元・先端インデックス
      b2, t2 = (i + 1) * 2, (i + 1) * 2 + 1  # 後ろのヒレの根元・先端インデックス
      skin_faces_l += (
          f" {b1} {t1} {t2}  {b1} {t2} {b2}"  # 左膜を構成するポリゴン2つ
      )
      skin_faces_r += (
          f" {b1} {t2} {t1}  {b1} {b2} {t2}"  # 右膜を構成するポリゴン2つ
      )

  # 全体の設定・流体密度・パーツ配置・ヒレ膜を統合したXMLテキストを出力
  return f"""
<mujoco>
  <compiler angle="degree"/>
  <!-- 重力なし(0 0 0)、流体密度(density=1000: 水相当)、粘性抵抗(viscosity=0.001) -->
  <option gravity="0 0 0" density="1000" viscosity="0.001"/>

  <asset>
    <!-- 胴体とヒレの3D CADデータ(.stl)をロード -->
    <mesh name="torso_mesh" file="assets/torso.stl" scale="0.001 0.001 0.001"/>
    <mesh name="fin_mesh" file="assets/fin.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0 0 3"/>
    <!-- 胴体パーツ（自由関節で浮遊） -->
    <body name="torso" pos="0 0 {Z_BODY}">
      <joint type="free"/>
      <geom type="mesh" mesh="torso_mesh" rgba="0.2 0.6 0.8 1" contype="0" conaffinity="0"/>
      {left_fins_xml}
      {right_fins_xml}
    </body>
  </worldbody>

  <!-- 変形可能なヒレ膜（スキン）の定義 -->
  <deformable>
    <skin name="membrane_left" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_l}" face="{skin_faces_l}">
      {skin_bones_l}
    </skin>
    <skin name="membrane_right" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_r}" face="{skin_faces_r}">
      {skin_bones_r}
    </skin>
  </deformable>

  <!-- モーターの設定 -->
  <actuator>
    {actuators_xml}
  </actuator>
</mujoco>
"""


# 動的に作成したXML文脈からMuJoCo物理モデルオブジェクトを作成
model = mujoco.MjModel.from_xml_string(generate_xml())
# 状態値（位置・速度・力など）を保存するデータオブジェクトを作成
data = mujoco.MjData(model)
# 胴体（torso）のIDを取得（位置や速度を追尾・記録するため）
torso_id = model.body("torso").id

# 実験データを蓄積する空のリストを用意
log_data = []

# 保存するCSVファイル名を波の数に応じて自動生成（例: sim_result_wave_2.0.csv）
#csv_filename = f"sim_result_wave_{NUMBER_OF_WAVES}.csv"
csv_filename = f"sim( {AMPLITUDE_DEG} _ {FREQ} _ {NUM_FINS} _ {NUMBER_OF_WAVES}).csv"
#csv_filename = f"sim(振れ幅:{AMPLITUDE_DEG} 上下運動の周波数:{FREQ} ヒレの数:{NUM_FINS} 形成する波の数:{NUMBER_OF_WAVES}).csv"
print(f"🚀 実験開始 (振れ幅:{AMPLITUDE_DEG} 上下運動の周波数:{FREQ} ヒレの数:{NUM_FINS} 形成する波の数:{NUMBER_OF_WAVES} ➔ 保存先: {csv_filename}")

try:
  # 3D描画用のインタラクティブビューワーを起動
  with mujoco.viewer.launch_passive(
      model, data, key_callback=key_callback
  ) as viewer:
    # 視点（カメラ）の位置や角度をセット
    viewer.cam.distance = 0.65  # カメラ距離
    viewer.cam.elevation = -30  # 俯瞰角度（上下）
    viewer.cam.azimuth = 135  # 方位角（左右）

    # 3Dウインドウが開いている間ループ処理を継続
    while viewer.is_running():
      step_start = time.time()  # 1ステップ処理の開始時刻を取得

      if not is_paused:  # 一時停止中でない場合のみ物理演算を進行
        t = data.time  # シミュレーション内の現在時刻[秒]を取得

        # 各ヒレの運動（正弦波の目標角度）を計算して入力
        for i in range(NUM_FINS):
          # 時刻tとヒレの位置iに応じた「位相（角度）」を算出
          phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)
          # sin関数を使って目標角度[度]を計算
          angle_deg = math.sin(phase) * AMPLITUDE_DEG
          # 単位を度(deg)からラジアン(rad)へ変換
          angle_rad = math.radians(angle_deg)

          # アクチュエータに目標角度を指示（右ヒレは左右線対称にするため符号を反転）
          data.ctrl[i * 2] = angle_rad  # 左ヒレi
          data.ctrl[i * 2 + 1] = -angle_rad  # 右ヒレi

        # 物理エンジンを1ステップ進める（流体抵抗や動力を計算）
        mujoco.mj_step(model, data)

        # カメラが常にアノマロカリスの胴体を追いかけるよう設定
        viewer.cam.lookat = data.xpos[torso_id]

        # 胴体の現在地（X軸位置[m]）と進行速度（X軸速度[cm/s]）を取得
        x_pos = data.xpos[torso_id][0]
        x_vel_cms = (
            data.cvel[torso_id][3] * 100.0
        )  # m/s から cm/s に単位換算（cvel[3]がX軸並進速度）
        # ログリストに [時刻, X位置, X速度] を追加
        log_data.append([t, x_pos, x_vel_cms])

      # 3D画面の表示を最新情報に更新
      viewer.sync()

      # シミュレーションの進行スピードを「実時間(SIM_SPEED)」と同期させるタイマー処理
      elapsed = time.time() - step_start  # 1ステップにかかった計算時間
      target_delay = (
          model.opt.timestep / SIM_SPEED
      )  # 理想的な待機時間（タイムステップ ÷ 倍速設定）
      if target_delay > elapsed:
        remaining = target_delay - elapsed
        while remaining > 0 and viewer.is_running():
          sleep_time = min(remaining, 0.01)
          time.sleep(sleep_time)  # 短時間ずつスリープして待機
          remaining -= sleep_time

except KeyboardInterrupt:
  # 実行中にキーボード（Ctrl+C）で停止された場合の例外ハンドリング
  pass

# 収集したデータをCSVファイルへ書き出し処理
with open(csv_filename, "w", newline="", encoding="utf-8") as f:
  writer = csv.writer(f)
  # 1行目にヘッダー（列名）を書き込み
  writer.writerow(["time", "pos_x", "vel_x_cms"])
  # 記録したデータを全行書き込み
  writer.writerows(log_data)

print(f"✅ CSV出力完了: {csv_filename}")
sys.exit(0)  # プログラムを正常終了