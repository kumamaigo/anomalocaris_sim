import math
import sys
import time

import mujoco
import mujoco.viewer


# ============================================================
# 動作調整パラメータ
# ============================================================

# ヒレの目標角度の振幅
# 実際にposition actuatorへ与える目標値は ±20°
AMPLITUDE_DEG = 20.0

# 周波数
# 1波 = 3秒
FREQ = 1.0 / 3.0

# シミュレーション速度
SIM_SPEED = 1.0

# ヒレ同士の位相差 [rad]
PHASE_SHIFT = 0.5


# ============================================================
# ヒレの配置
# ============================================================

X_START = 0.110
X_STEP = 0.055

Y_LEFT = 0.030
Y_RIGHT = -0.030

Z_BODY = 1.0

FIN_LENGTH = 0.12


# ============================================================
# Joint / Controller パラメータ
# ============================================================

# Jointの物理的な可動範囲
#
# 目標値は ±20°だが、
# 万一制御によってオーバーシュートしても
# ±45°より外側には行かないようにする。
JOINT_LIMIT_DEG = 45.0

# Position actuatorのPゲイン
#
# 大きすぎると強く動かしすぎて
# オーバーシュートしやすくなる。
KP = 2.0

# Joint damping
#
# 大きくすると振動・オーバーシュートを抑えやすい。
DAMPING = 0.5

# Armature
ARMATURE = 0.005


# ============================================================
# MuJoCo XML生成
# ============================================================

def generate_xml():

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

        # ----------------------------------------------------
        # ヒレのX位置
        # ----------------------------------------------------

        x_pos = X_START - (i * X_STEP)


        # ----------------------------------------------------
        # 左ヒレ
        # ----------------------------------------------------

        left_fins_xml += f"""
        <body
            name="fin_l{i}"
            pos="{x_pos:.4f} {Y_LEFT:.4f} 0">

            <joint
                name="joint_l{i}"
                type="hinge"
                axis="1 0 0"
                range="-{JOINT_LIMIT_DEG} {JOINT_LIMIT_DEG}"
                limited="true"
                armature="{ARMATURE}"
                damping="{DAMPING}"/>

            <geom
                type="mesh"
                mesh="fin_mesh"
                rgba="0.8 0.3 0.2 1"
                contype="0"
                conaffinity="0"/>
        </body>
        """


        # ----------------------------------------------------
        # 右ヒレ
        # ----------------------------------------------------

        right_fins_xml += f"""
        <body
            name="fin_r{i}"
            pos="{x_pos:.4f} {Y_RIGHT:.4f} 0">

            <joint
                name="joint_r{i}"
                type="hinge"
                axis="1 0 0"
                range="-{JOINT_LIMIT_DEG} {JOINT_LIMIT_DEG}"
                limited="true"
                armature="{ARMATURE}"
                damping="{DAMPING}"/>

            <geom
                type="mesh"
                mesh="fin_mesh"
                euler="0 0 180"
                rgba="0.2 0.8 0.3 1"
                contype="0"
                conaffinity="0"/>
        </body>
        """


        # ----------------------------------------------------
        # Position actuator
        #
        # Pythonから与えるdata.ctrlは「目標角度」
        # 単位はrad
        # ----------------------------------------------------

        actuators_xml += f"""
        <position
            name="actuator_l{i}"
            joint="joint_l{i}"
            kp="{KP}"/>

        <position
            name="actuator_r{i}"
            joint="joint_r{i}"
            kp="{KP}"/>
        """


        # ====================================================
        # 左側 membrane
        # ====================================================

        v_base_l = (
            f" {x_pos:.4f}"
            f" {Y_LEFT:.4f}"
            f" {Z_BODY:.4f}"
        )

        v_tip_l = (
            f" {x_pos:.4f}"
            f" {Y_LEFT + FIN_LENGTH:.4f}"
            f" {Z_BODY:.4f}"
        )

        skin_vertices_l += v_base_l + v_tip_l


        skin_bones_l += f"""
        <bone
            body="fin_l{i}"
            bindpos="{x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}"
            bindquat="1 0 0 0"
            vertid="{i * 2} {i * 2 + 1}"
            vertweight="1.0 1.0"/>
        """


        # ====================================================
        # 右側 membrane
        # ====================================================

        v_base_r = (
            f" {x_pos:.4f}"
            f" {Y_RIGHT:.4f}"
            f" {Z_BODY:.4f}"
        )

        v_tip_r = (
            f" {x_pos:.4f}"
            f" {Y_RIGHT - FIN_LENGTH:.4f}"
            f" {Z_BODY:.4f}"
        )

        skin_vertices_r += v_base_r + v_tip_r


        skin_bones_r += f"""
        <bone
            body="fin_r{i}"
            bindpos="{x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}"
            bindquat="1 0 0 0"
            vertid="{i * 2} {i * 2 + 1}"
            vertweight="1.0 1.0"/>
        """


        # ====================================================
        # membraneの面
        # ====================================================

        if i < 5:

            b1 = i * 2
            t1 = i * 2 + 1

            b2 = (i + 1) * 2
            t2 = (i + 1) * 2 + 1


            # 左
            skin_faces_l += (
                f" {b1} {t1} {t2}"
                f" {b1} {t2} {b2}"
            )


            # 右
            skin_faces_r += (
                f" {b1} {t2} {t1}"
                f" {b1} {b2} {t2}"
            )


    # ========================================================
    # XML本体
    # ========================================================

    return f"""
<mujoco>

    <compiler angle="degree"/>

    <option
        gravity="0 0 0"
        timestep="0.002"/>


    <!-- ================================================== -->
    <!-- Assets -->
    <!-- ================================================== -->

    <asset>

        <mesh
            name="torso_mesh"
            file="assets/torso.stl"
            scale="0.001 0.001 0.001"/>

        <mesh
            name="fin_mesh"
            file="assets/fin.stl"
            scale="0.001 0.001 0.001"/>

    </asset>


    <!-- ================================================== -->
    <!-- Robot body -->
    <!-- ================================================== -->

    <worldbody>

        <light
            pos="0 0 3"/>


        <body
            name="torso"
            pos="0 0 {Z_BODY}">

            <geom
                type="mesh"
                mesh="torso_mesh"
                rgba="0.2 0.6 0.8 1"
                contype="0"
                conaffinity="0"/>


            {left_fins_xml}

            {right_fins_xml}

        </body>

    </worldbody>


    <!-- ================================================== -->
    <!-- Deformable membrane -->
    <!-- ================================================== -->

    <deformable>

        <skin
            name="membrane_left"
            rgba="0.1 0.8 0.9 0.6"
            vertex="{skin_vertices_l}"
            face="{skin_faces_l}">

            {skin_bones_l}

        </skin>


        <skin
            name="membrane_right"
            rgba="0.1 0.8 0.9 0.6"
            vertex="{skin_vertices_r}"
            face="{skin_faces_r}">

            {skin_bones_r}

        </skin>

    </deformable>


    <!-- ================================================== -->
    <!-- Actuators -->
    <!-- ================================================== -->

    <actuator>

        {actuators_xml}

    </actuator>

</mujoco>
"""


# ============================================================
# MuJoCo Model生成
# ============================================================

xml = generate_xml()

model = mujoco.MjModel.from_xml_string(xml)

data = mujoco.MjData(model)


# ============================================================
# 起動メッセージ
# ============================================================

print()
print("==============================================")
print("  Fin simulation")
print("==============================================")
print(f"Target amplitude : ±{AMPLITUDE_DEG:.1f} deg")
print(f"Joint limit      : ±{JOINT_LIMIT_DEG:.1f} deg")
print(f"Frequency        : {FREQ:.3f} Hz")
print(f"Period           : {1.0 / FREQ:.2f} sec")
print(f"KP               : {KP}")
print(f"Damping          : {DAMPING}")
print("==============================================")
print()


# ============================================================
# Viewer起動
# ============================================================

try:

    with mujoco.viewer.launch_passive(model, data) as viewer:

        # ----------------------------------------------------
        # カメラ
        # ----------------------------------------------------

        viewer.cam.lookat = [
            0.0,
            0.0,
            Z_BODY
        ]

        viewer.cam.distance = 0.65

        viewer.cam.elevation = -30

        viewer.cam.azimuth = 135


        # ----------------------------------------------------
        # メインループ
        # ----------------------------------------------------

        while viewer.is_running():

            step_start = time.time()


            # =================================================
            # 現在時刻
            # =================================================

            t = data.time


            # =================================================
            # 6枚のヒレを制御
            # =================================================

            for i in range(6):

                # ---------------------------------------------
                # 位相
                # ---------------------------------------------

                phase = (
                    2.0
                    * math.pi
                    * FREQ
                    * t
                    - i * PHASE_SHIFT
                )


                # ---------------------------------------------
                # 目標角度 [deg]
                #
                # -20° ～ +20°
                # ---------------------------------------------

                target_angle_deg = (
                    math.sin(phase)
                    * AMPLITUDE_DEG
                )


                # ---------------------------------------------
                # MuJoCoへ渡すためradへ変換
                # ---------------------------------------------

                target_angle_rad = math.radians(
                    target_angle_deg
                )


                # ---------------------------------------------
                # 左右へ逆方向の目標値
                # ---------------------------------------------

                left_actuator = i * 2

                right_actuator = i * 2 + 1


                data.ctrl[left_actuator] = (
                    target_angle_rad
                )

                data.ctrl[right_actuator] = (
                    -target_angle_rad
                )


            # =================================================
            # 物理シミュレーション
            # =================================================

            mujoco.mj_step(model, data)


            # =================================================
            # Viewer更新
            # =================================================

            viewer.sync()


            # =================================================
            # シミュレーション速度調整
            # =================================================

            elapsed = time.time() - step_start

            target_delay = (
                model.opt.timestep
                / SIM_SPEED
            )

            if target_delay > elapsed:

                remaining = (
                    target_delay
                    - elapsed
                )

                while (
                    remaining > 0
                    and viewer.is_running()
                ):

                    sleep_time = min(
                        remaining,
                        0.01
                    )

                    time.sleep(
                        sleep_time
                    )

                    remaining -= sleep_time


except KeyboardInterrupt:

    pass


# ============================================================
# 終了
# ============================================================

print()
print("==============================================")
print("  Simulation finished")
print("==============================================")

sys.exit(0)