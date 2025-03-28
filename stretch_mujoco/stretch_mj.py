import os
import cv2
import mujoco
import mujoco.viewer
from mujoco.glfw import *
import numpy as np
import threading

import utils


# model = mujoco.MjModel.from_xml_path('/Users/ingui/Documents/stretch_mujoco/stretch_mujoco/models/assets/house2/house2/house2.xml')
# data = mujoco.MjData(model)

## Load the scene and spawn the robot
mjspec = mujoco.MjSpec.from_file('./stretch_mujoco/models/pc_scene.xml')
# mjspec = mujoco.MjSpec.from_file('/Users/ingui/Documents/stretch_mujoco/stretch_mujoco/models/assets/worlds/00010-DBjEcHFg4oq/house_00010/house_00010.xml')
model = mjspec.compile()
xml = mjspec.to_xml()

# robot_spawn = {
#     "pos": "2.5 -1.2 0.0",
#     "quat": "1 0 0 0"
# }

robot_spawn = {
    "pos": "0.0 0.0 0.1",
    "quat": "1 0 0 0"
}

stretch_xml_absolute = utils.get_absolute_path_stretch_xml(robot_spawn)
xml = utils.insert_line_after_mujoco_tag(
    xml,
    f' <include file="{stretch_xml_absolute}"/>',
)
# xml = utils.insert_line_after_asset_tag(
#     xml,
#     f' <model name="'
# )

# with open('temp.xml', 'w') as f:
#     f.write(xml)

mjspec = mujoco.MjSpec.from_string(xml)
# mjspec = mujoco.MjSpec.from_file('./stretch_mujoco/models/assets/worlds/house2/house2/house2.xml')
model = mjspec.compile()

# with open('temp.xml', 'w') as f:
#     f.write(mjspec.to_xml())

data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco.viewer

# simulate and render
def run():
    viewer.launch(model, data, show_left_ui=True, show_right_ui=True)

# threading.Thread(target=run, name="mujoco_viewer_thread").start()

try:
    run()
    while True:
        mujoco.simulate(model, data)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

except KeyboardInterrupt:
    os.kill(os.getpid(), 9)
# close
viewer.close()
