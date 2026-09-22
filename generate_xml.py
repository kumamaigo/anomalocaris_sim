from xml.dom import minidom
import xml.etree.ElementTree as ET


def create_rod_model():
    mujoco = ET.Element("mujoco", model="anomalocaris_rods")

    ET.SubElement(
        mujoco, "compiler", angle="degree", coordinate="local"
    )
    ET.SubElement(
        mujoco,
        "option",
        timestep="0.001",
        gravity="0 0 0",
        density="1000",
        viscosity="0.002",
    )

    worldbody = ET.SubElement(mujoco, "worldbody")
    actuators = ET.SubElement(mujoco, "actuator")

    # 照明
    ET.SubElement(
        worldbody,
        "light",
        directional="true",
        diffuse="0.8 0.8 0.8",
        pos="0 0 3",
        dir="0 0 -1",
    )

    # 黒い本体 (直方体)
    torso = ET.SubElement(worldbody, "body", name="torso", pos="0 0 0")
    ET.SubElement(
        torso,
        "geom",
        type="box",
        size="0.25 0.04 0.03",
        rgba="0.2 0.2 0.2 1",
        mass="1.0",
    )

    # 左右6本ずつの赤い棒 (カプセル)
    NUM_RODS = 6
    ROD_LENGTH = 0.15  # 15cm
    X_START = 0.2
    X_END = -0.2

    for i in range(NUM_RODS):
        # 頭から尾へX座標を配置
        x_pos = X_START - (i * (X_START - X_END) / (NUM_RODS - 1))

        # 左側の棒 (+Y方向)
        body_left = ET.SubElement(
            torso, "body", name=f"rod_L_{i+1}", pos=f"{x_pos:.3f} 0.04 0"
        )
        ET.SubElement(
            body_left,
            "joint",
            name=f"joint_L_{i+1}",
            type="hinge",
            pos="0 0 0",
            axis="1 0 0",
            range="-45 45",
            damping="0.05",
        )
        ET.SubElement(
            body_left,
            "geom",
            type="capsule",
            fromto=f"0 0 0 0 {ROD_LENGTH} 0",
            size="0.005",
            rgba="1 0.2 0.2 1",
            mass="0.01",
        )
        ET.SubElement(
            actuators,
            "position",
            name=f"motor_L_{i+1}",
            joint=f"joint_L_{i+1}",
            kp="10.0",
        )

        # 右側の棒 (-Y方向)
        body_right = ET.SubElement(
            torso, "body", name=f"rod_R_{i+1}", pos=f"{x_pos:.3f} -0.04 0"
        )
        ET.SubElement(
            body_right,
            "joint",
            name=f"joint_R_{i+1}",
            type="hinge",
            pos="0 0 0",
            axis="1 0 0",
            range="-45 45",
            damping="0.05",
        )
        ET.SubElement(
            body_right,
            "geom",
            type="capsule",
            fromto=f"0 0 0 0 {-ROD_LENGTH} 0",
            size="0.005",
            rgba="1 0.2 0.2 1",
            mass="0.01",
        )
        ET.SubElement(
            actuators,
            "position",
            name=f"motor_R_{i+1}",
            joint=f"joint_R_{i+1}",
            kp="10.0",
        )

    xml_str = minidom.parseString(ET.tostring(mujoco)).toprettyxml(indent="  ")
    with open("rod_model.xml", "w") as f:
        f.write(xml_str)

    print(
        "✅ 本体＋左右6本の棒モデル (rod_model.xml) を生成しました！"
    )


if __name__ == "__main__":
    create_rod_model()