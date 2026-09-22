import math
import sys
import time
import mujoco
import mujoco.viewer

# ==========================================
# 📍 動作調整用パラメータ（単位：度）
# ==========================================
# ヒレの片側振り幅（15度）。上へ15度、下へ15度動きます（全可動幅30度）
AMPLITUDE = 15.0

# 1波（1往復）にかける時間＝3秒（周波数 1/3 Hz）
FREQ = 1.0 / 3.0

# シミュレーションの進行速度（1.0 ＝ 等倍のリアルタイム速）
SIM_SPEED = 1.0

# 前後のヒレの位相差（うねり波のズレ具合）
PHASE_SHIFT = 0.5

# 配置用パラメータ（メートル単位）
X_START = 0.110  # 先頭ヒレのX位置
X_STEP = 0.055  # ヒレ同士の間隔
Y_LEFT = 0.030  # 左ヒレの軸位置
Y_RIGHT = -0.030  # 右ヒレの軸位置
Z_BODY = 1.0  # 胴体の高さ
FIN_LENGTH = 0.12  # ヒレの長さ

# 💡 追加：制御ゲイン関連パラメータ（発振・レンジ超過対策）
JOINT_DAMPING = 1.5     # 関節ダンピング（0.5→1.5に強化）
JOINT_ARMATURE = 0.005  # 人工慣性（ヒレの実慣性が小さすぎる問題への対策）
ACT_KP = 15.0           # 位置ゲイン（30→15に低減）
ACT_KV = 3.0            # 速度ゲイン（新規追加：発振を直接抑制）
SIM_TIMESTEP = 0.001    # タイムステップを明示（デフォルトより細かく）
# ==========================================


def generate_xml():
    """MuJoCo用のモデル構造（XML）を動的に生成する関数"""
    left_fins_xml = ""
    right_fins_xml = ""
    actuators_xml = ""

    skin_vertices_l = ""
    skin_vertices_r = ""
    skin_bones_l = ""
    skin_bones_r = ""
    skin_faces_l = ""
    skin_faces_r = ""

    for i in range(6):
        x_pos = X_START - (i * X_STEP)

        # 💡 左ヒレの関節
        # ・limited="true" を明示（rangeが確実に物理拘束として有効になるように）
        # ・damping を強化
        # ・armature を追加（低慣性による発振対策）
        left_fins_xml += f"""
      <body name="fin_l{i}" pos="{x_pos:.4f} {Y_LEFT:.4f} 0">
        <joint name="joint_l{i}" type="hinge" axis="1 0 0"
               range="-45 45" limited="true"
               damping="{JOINT_DAMPING}" armature="{JOINT_ARMATURE}"/>
        <geom type="mesh" mesh="fin_mesh" rgba="0.8 0.3 0.2 1" contype="0" conaffinity="0"/>
      </body>"""

        # 💡 右ヒレの関節（同様の修正）
        right_fins_xml += f"""
      <body name="fin_r{i}" pos="{x_pos:.4f} {Y_RIGHT:.4f} 0">
        <joint name="joint_r{i}" type="hinge" axis="1 0 0"
               range="-45 45" limited="true"
               damping="{JOINT_DAMPING}" armature="{JOINT_ARMATURE}"/>
        <geom type="mesh" mesh="fin_mesh" euler="0 0 180" rgba="0.2 0.8 0.3 1" contype="0" conaffinity="0"/>
      </body>"""

        # 💡 アクチュエータに kv（速度フィードバックゲイン）を追加し、kpを下げて発振を抑制
        actuators_xml += f"""
    <position joint="joint_l{i}" kp="{ACT_KP}" kv="{ACT_KV}" ctrlrange="-45 45" ctrllimited="true"/>
    <position joint="joint_r{i}" kp="{ACT_KP}" kv="{ACT_KV}" ctrlrange="-45 45" ctrllimited="true"/>"""

        # 膜（スキン）の頂点と骨格の関連付け（変更なし）
        v_base_l = f" {x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}"
        v_tip_l = f" {x_pos:.4f} {Y_LEFT + FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_l += v_base_l + v_tip_l
        skin_bones_l += f"""
      <bone body="fin_l{i}" bindpos="{x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        v_base_r = f" {x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}"
        v_tip_r = f" {x_pos:.4f} {Y_RIGHT - FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_r += v_base_r + v_tip_r
        skin_bones_r += f"""
      <bone body="fin_r{i}" bindpos="{x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        if i < 5:
            b1, t1 = i * 2, i * 2 + 1
            b2, t2 = (i + 1) * 2, (i + 1) * 2 + 1
            skin_faces_l += f" {b1} {t1} {t2}  {b1} {t2} {b2}"
            skin_faces_r += f" {b1} {t2} {t1}  {b1} {b2} {t2}"

    return f"""
<mujoco>
  <!-- compiler angle="degree" により、モデル全体（ctrl含む）の角度単位を「度」に統一 -->
  <compiler angle="degree"/>
  <!-- 💡 timestepを明示し、integratorをimplicitfastに変更（低慣性・高ゲイン系の発振対策） -->
  <option gravity="0 0 0" timestep="{SIM_TIMESTEP}" integrator="implicitfast"/>

  <asset>
    <mesh name="torso_mesh" file="assets/torso.stl" scale="0.001 0.001 0.001"/>
    <mesh name="fin_mesh" file="assets/fin.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0 0 3"/>
    <body name="torso" pos="0 0 {Z_BODY}">
      <geom type="mesh" mesh="torso_mesh" rgba="0.2 0.6 0.8 1" contype="0" conaffinity="0"/>
      {left_fins_xml}
      {right_fins_xml}
    </body>
  </worldbody>

  <deformable>
    <skin name="membrane_left" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_l}" face="{skin_faces_l}">
      {skin_bones_l}
    </skin>
    <skin name="membrane_right" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_r}" face="{skin_faces_r}">
      {skin_bones_r}
    </skin>
  </deformable>

  <actuator>
    {actuators_xml}
  </actuator>
</mujoco>
"""


# モデルと状態データの作成
model = mujoco.MjModel.from_xml_string(generate_xml())
data = mujoco.MjData(model)

print("🚀 減衰追加・安定化版シミュレーションを開始します...")

# 💡 デバッグ用：各ヒレの質量を起動時に一度だけ表示（極端に軽い場合の切り分け用）
for i in range(6):
    body_id_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"fin_l{i}")
    print(f"fin_l{i} mass: {model.body_mass[body_id_l]:.6f} kg")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat = [0.0, 0.0, Z_BODY]
        viewer.cam.distance = 0.65
        viewer.cam.elevation = -30
        viewer.cam.azimuth = 135

        # 💡 デバッグ表示用のタイマー
        last_print_time = 0.0

        while viewer.is_running():
            step_start = time.time()

            t = data.time
            for i in range(6):
                # 波の位相計算（3秒で1周）
                phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)

                # 目標角度（度）の計算（-15度 〜 +15度）
                angle_deg = math.sin(phase) * AMPLITUDE

                # compiler angle="degree" のため、「度」の値をそのまま入力
                data.ctrl[i * 2] = angle_deg  # 左ヒレ
                data.ctrl[i * 2 + 1] = -angle_deg  # 右ヒレ

            # 物理シミュレーションを1ステップ更新
            mujoco.mj_step(model, data)
            viewer.sync()

            # 💡 デバッグ用：1秒ごとに目標角と実角度(joint_l0)を比較表示
            if t - last_print_time >= 1.0:
                target = math.sin(2 * math.pi * FREQ * t) * AMPLITUDE
                actual = math.degrees(data.qpos[0])
                print(f"t={t:.1f}s  target_l0={target:6.1f}°  actual_l0={actual:6.1f}°")
                last_print_time = t

            # リアルタイム同期（実時間待機）処理
            elapsed = time.time() - step_start
            target_delay = model.opt.timestep / SIM_SPEED
            if target_delay > elapsed:
                remaining = target_delay - elapsed
                while remaining > 0 and viewer.is_running():
                    sleep_time = min(remaining, 0.01)
                    time.sleep(sleep_time)
                    remaining -= sleep_time

except KeyboardInterrupt:
    pass

print("🛑 シミュレーションを正常に終了しました。")
sys.exit(0)