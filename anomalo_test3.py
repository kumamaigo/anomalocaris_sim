import math  # 三角関数（sin）や角度・ラジアン変換を行うための標準ライブラリ
import sys  # プログラムを安全に終了（sys.exit）させるための標準ライブラリ
import time  # ループの実行時間を計測し、シミュレーション速度を調整するためのライブラリ
import mujoco  # MuJoCo 物理計算エンジンのメインライブラリ
import mujoco.viewer  # 3Dモデルを画面に表示・操作するための描画用モジュール

# ==========================================
# 📍 動作調整用パラメータ
# ==========================================
AMPLITUDE_DEG = 20.0  # ヒレの振幅（片側に最大20度まで傾く）
FREQ = 0.5  # 波の周波数（1.0Hz ＝ 1秒間に1サイクルのペースで波打つ:小さいほどゆっくり）
SIM_SPEED = 1.0  # シミュレーションの再生速度（1.0 で現実と同じ等倍速）

NUMBER_OF_WAVES = 1.0  # 体全体に同時に発生させたい「波の個数」（2つの波を表示）
PHASE_SHIFT = (
    2 * math.pi * NUMBER_OF_WAVES
) / 5  # 6枚のヒレ（間隔5箇所）で2波長（4π）作るためのヒレ同士の位相差

X_START = 0.110  # 一番前のヒレ（0番目）のX座標位置（メートル）
X_STEP = 0.055  # 前後のヒレ同士のX軸方向の間隔（5.5cm刻み）
Y_LEFT = 0.030  # 左側のヒレを取り付けるY座標位置（中心から左へ3cm）
Y_RIGHT = -0.030  # 右側のヒレを取り付けるY座標位置（中心から右へ3cm）
Z_BODY = 1.0  # アノマロカリス全体の高さ（Z座標 1.0mの位置に配置）
FIN_LENGTH = 0.12  # 膜（スキン）を描画するためのヒレの長さ（12cm）
# ==========================================

# 💡 一時停止の状態を管理する変数（True: 停止中, False: 動いている）
is_paused = False


def key_callback(keycode):
    """3Dビューアー上でキーボードが押されたときに自動で呼ばれる関数"""
    global is_paused  # 関数の中から外側の is_paused 変数を書き換えられるようにする
    if keycode == 32:  # キーコード「32」は Space キーを意味する
        is_paused = not is_paused  # 停止状態を反転させる（TrueならFalse、FalseならTrue）
        status = "一時停止 ⏸️" if is_paused else "再開 ▶️"  # コンソール表示用の文字設定
        print(f"[{status}] (Spaceキーが押されました)")  # 状態が変わったことをターミナルに表示


def generate_xml():
    """MuJoCo用のモデル構造（XMLテキスト）を自動生成する関数"""
    left_fins_xml = ""  # 左ヒレ群のXMLコードを入れる空の文字列を作成
    right_fins_xml = ""  # 右ヒレ群のXMLコードを入れる空の文字列を作成
    actuators_xml = ""  # モーター（アクチュエータ）のXMLコードを入れる空の文字列を作成

    skin_vertices_l = ""  # 左側の皮膜の頂点座標データを保持する文字列
    skin_vertices_r = ""  # 右側の皮膜の頂点座標データを保持する文字列
    skin_bones_l = ""  # 左側の皮膜とヒレ関節を結びつけるボーンデータ
    skin_bones_r = ""  # 右側の皮膜とヒレ関節を結びつけるボーンデータ
    skin_faces_l = ""  # 左側の皮膜の表面（三角形の面）を構成するインデックスデータ
    skin_faces_r = ""  # 右側の皮膜の表面（三角形の面）を構成するインデックスデータ

    # 6対（12枚）のヒレと皮膜の構造をループ処理で順番に生成する
    for i in range(6):
        x_pos = X_START - (i * X_STEP)  # i番目のヒレのX位置を計算

        # --- 左側のヒレ（body, joint, geom）のXMLを作成 ---
        left_fins_xml += f"""
      <body name="fin_l{i}" pos="{x_pos:.4f} {Y_LEFT:.4f} 0">
        <joint name="joint_l{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" rgba="0.8 0.3 0.2 1" contype="0" conaffinity="0"/>
      </body>"""

        # --- 右側のヒレ（body, joint, geom）のXMLを作成（180度回転して反対側に向ける）---
        right_fins_xml += f"""
      <body name="fin_r{i}" pos="{x_pos:.4f} {Y_RIGHT:.4f} 0">
        <joint name="joint_r{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" euler="0 0 180" rgba="0.2 0.8 0.3 1" contype="0" conaffinity="0"/>
      </body>"""

        # --- 各関節を動かす位置制御モーター（P制御ゲイン kp=5）のXMLを作成 ---
        actuators_xml += f"""
    <position joint="joint_l{i}" kp="5"/>
    <position joint="joint_r{i}" kp="5"/>"""

        # --- 左皮膜の根元と先端の3D座標文字列を生成 ---
        v_base_l = f" {x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}"
        v_tip_l = f" {x_pos:.4f} {Y_LEFT + FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_l += v_base_l + v_tip_l  # 頂点リストに結合
        # 左ヒレの関節に皮膜の頂点を追従（バインド）させる設定
        skin_bones_l += f"""
      <bone body="fin_l{i}" bindpos="{x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        # --- 右皮膜の根元と先端の3D座標文字列を生成 ---
        v_base_r = f" {x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}"
        v_tip_r = f" {x_pos:.4f} {Y_RIGHT - FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_r += v_base_r + v_tip_r  # 頂点リストに結合
        # 右ヒレの関節に皮膜の頂点を追従させる設定
        skin_bones_r += f"""
      <bone body="fin_r{i}" bindpos="{x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        # 前後のヒレの頂点同士を繋ぎ、四角形（＝三角形2枚）のメッシュ面を作る
        if i < 5:
            b1, t1 = i * 2, i * 2 + 1  # 現在のヒレの根元と先端の頂点番号
            b2, t2 = (i + 1) * 2, (i + 1) * 2 + 1  # 1つ後ろのヒレの根元と先端の頂点番号
            skin_faces_l += (
                f" {b1} {t1} {t2}  {b1} {t2} {b2}"  # 表裏両方から見えるように面を貼る（左）
            )
            skin_faces_r += (
                f" {b1} {t2} {t1}  {b1} {b2} {t2}"  # 表裏両方から見えるように面を貼る（右）
            )

    # 組み立てたパーツ情報を一つの大きな XML (MJCF) フォーマット文字列として返却する
    return f"""
<mujoco>
  <compiler angle="degree"/> <!-- XML内での角度表現を「度」に設定 -->
  <option gravity="0 0 0"/> <!-- 無重力空間に設定（水中の浮力拮抗を表現） -->

  <asset>
    <!-- 外部アセット（STL 3Dモデル）の読み込みと拡大縮小率の設定 -->
    <mesh name="torso_mesh" file="assets/torso.stl" scale="0.001 0.001 0.001"/>
    <mesh name="fin_mesh" file="assets/fin.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0 0 3"/> <!-- 空間を照らす照明の配置 -->
    <body name="torso" pos="0 0 {Z_BODY}"> <!-- 胴体パーツの配置 -->
      <geom type="mesh" mesh="torso_mesh" rgba="0.2 0.6 0.8 1" contype="0" conaffinity="0"/>
      {left_fins_xml}  <!-- 生成した左ヒレ群を結合 -->
      {right_fins_xml} <!-- 生成した右ヒレ群を結合 -->
    </body>
  </worldbody>

  <deformable>
    <!-- ヒレ同士を結ぶ「ひらひらした可変膜（スキン）」の設定 -->
    <skin name="membrane_left" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_l}" face="{skin_faces_l}">
      {skin_bones_l}
    </skin>
    <skin name="membrane_right" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_r}" face="{skin_faces_r}">
      {skin_bones_r}
    </skin>
  </deformable>

  <actuator>
    {actuators_xml} <!-- 生成した全モーター（12個）を配置 -->
  </actuator>
</mujoco>
"""


# --- メイン処理の開始 ---
# 1. XML文字列から、変更されない静的な設計図データ (MjModel) を構築
model = mujoco.MjModel.from_xml_string(generate_xml())
# 2. 設計図データをもとに、刻一刻と変化する状態データ (MjData) を作成
data = mujoco.MjData(model)

print("🚀 シミュレーションを開始します... [Spaceキーで一時停止/再開]")

try:
    # 3Dビューアーを起動（キー入力を監視する key_callback を登録）
    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        # 起動時のカメラの注視点と視点角度を設定
        viewer.cam.lookat = [0.0, 0.0, Z_BODY]  # カメラの注視点（胴体の位置）
        viewer.cam.distance = 0.65  # カメラからの距離（メートル）
        viewer.cam.elevation = -30  # 俯瞰（見下ろし）角度
        viewer.cam.azimuth = 135  # 方角角度

        # 3Dウィンドウが開いている間、無限ループでシミュレーションを実行する
        while viewer.is_running():
            step_start = time.time()  # 1ステップの計算開始時刻を記録

            # 💡 一時停止（is_paused）でない時だけ、関節の目標角度計算と物理計算を進める
            if not is_paused:
                t = data.time  # シミュレーション内の現在時刻（秒）を取得

                # 6対のヒレそれぞれに位相をズラした正弦波（sin波）を計算して与える
                for i in range(6):
                    # i番目のヒレの波の位相（タイミング）を計算
                    phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)

                    # sin関数から目標角度（度）を求め、Python用のラジアンに変換
                    angle_deg = math.sin(phase) * AMPLITUDE_DEG
                    angle_rad = math.radians(angle_deg)

                    # 左右のアクチュエータ（モーター）へ目標角度を設定（右側は反転運動）
                    data.ctrl[i * 2] = angle_rad  # 左側のモーターへ入力
                    data.ctrl[i * 2 + 1] = -angle_rad  # 右側のモーターへ入力

                # 物理時間を1ステップ（標準で0.002秒）だけ進める
                mujoco.mj_step(model, data)

            # 最新の物理状態を3D描画画面（ビューアー）に反映する
            viewer.sync()

            # --- シミュレーション速度を現実の時間経過と同期（等倍速化）させる処理 ---
            elapsed = time.time() - step_start  # 1ステップの処理にかかった実時間を計算
            target_delay = (
                model.opt.timestep / SIM_SPEED
            )  # 1ステップが進むべき目標の実時間
            if target_delay > elapsed:
                remaining = target_delay - elapsed  # 処理が早く終わりすぎた余り時間
                # 余り時間の間、ビューアーの操作性を損なわないよう細かく刻んで sleep（待機）する
                while remaining > 0 and viewer.is_running():
                    sleep_time = min(remaining, 0.01)
                    time.sleep(sleep_time)
                    remaining -= sleep_time

except KeyboardInterrupt:
    # ターミナルで Ctrl+C が押された場合はエラーを出さずに受け流す
    pass

print("🛑 シミュレーションを正常に終了しました。")
sys.exit(0)  # プログラムを終了して制御をOSに戻す