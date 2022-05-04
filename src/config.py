from easydict import EasyDict as edict

BOT_NAME = 'bishoybot'
cfg = edict({
    'camera_topic' : f"/{BOT_NAME}/camera_node/image/compressed",
    'wheels_topic' : f"/{BOT_NAME}/wheels_driver_node/wheels_cmd",
}) 