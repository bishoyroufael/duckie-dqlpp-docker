from easydict import EasyDict as edict

BOT_NAME = 'bishoybot'
cfg = edict({
    'camera_topic' : f"/{BOT_NAME}/camera_node/image/compressed",
    'wheels_topic' : f"/{BOT_NAME}/wheels_driver_node/wheels_cmd",
    'tof_topic' : f"/{BOT_NAME}/front_center_tof_driver_node/range",
    'camera_param_res' : [f"/{BOT_NAME}/camera_node/res_w", f"/{BOT_NAME}/camera_node/res_h"],
    'camera_width': 320,
    'camera_height': 240,
}) 