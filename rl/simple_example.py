import jax
import mujoco
from mujoco import mjx
import time
XML=r"""
<mujoco>
  <worldbody>
    <body>
      <freejoint/>
      <geom size=".15" mass="1" type="sphere"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
mjx_model = mjx.device_put(model)
global pos
pos=[None]*13
@jax.vmap
def batched_step(vel):
  mjx_data = mjx.make_data(mjx_model)
  qvel = mjx_data.qvel.at[0].set(vel)
  mjx_data = mjx_data.replace(qvel=qvel)
  global pos
  for i in range(0,12):
    pos[i]=mjx.step(mjx_model, mjx_data)

  # pos = mjx.step(mjx_model, mjx_data).qpos[0]
  return mean(pos)

vel = jax.numpy.arange(0.0, 100.0, 0.0001)
begin=time.time()
pos = jax.jit(batched_step)(vel)
end=time.time()
# print(pos)
print(len(vel)/1e6)

print(end-begin)
