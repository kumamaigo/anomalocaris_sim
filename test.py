import time
import mujoco
import mujoco.viewer

# 物理世界の設定（カメラ位置調整 + 重力で動くボール）
xml = """
<mujoco>
  <statistic center="0 0 1" extent="3"/>
  <worldbody>
    <light pos="0 0 3"/>
    <!-- 床 -->
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.8 0.8 1"/>
    
    <!-- 重力で落ちる赤いボール -->
    <body name="ball" pos="0 0 2">
      <freejoint/>
      <geom type="sphere" size="0.2" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, 0.002 - (time.time() - step_start)))