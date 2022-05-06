from easydict import EasyDict as edict

BOT_NAME = 'bishoybot'
cfg = edict({
    'camera_topic' : f"/{BOT_NAME}/camera_node/image/compressed",
    'wheels_topic' : f"/{BOT_NAME}/wheels_driver_node/wheels_cmd",
    'tof_topic' : f"/{BOT_NAME}/front_center_tof_driver_node/range",
    'raw_depth_topic': '/raw_depth',
    'camera_param_res' : [f"/{BOT_NAME}/camera_node/res_w", f"/{BOT_NAME}/camera_node/res_h"],
    'camera_width': 320,
    'camera_height': 240,
    'bot_speed': 0.1,
    'max_bot_delta_speed' : 0.1,
    'rotation_speed' : 0.5,
    'reverse_speed' : 0.4,
    'tof_min_range': 0.0,
    'tof_max_range': 1.2,
    'tof_min_range_threshold' : 0.20,
    'depth_dim' : 243,
    'training': {
        'reward_threshold' : 3000,
        'seed' : 0,
        'buffer_size' : 200,
        'actor_lr' : 1e-4,
        'critic_lr' : 1e-3,
        'gamma' : 0.99,
        'tau' : 0.005,
        'exploration_noise' : 0.1,
        'epoch' : 3,
        'step_per_epoch' : 200,
        'step_per_collect' : 8,
        'update_per_step' : 0.125,
        'batch_size' : 3,
        'hidden_sizes' : [32,32],
        'training_num' : 1,
        'test_num' : 1,
        'logdir' : 'log',
        'rew_norm' : 'store_true',
        'n_step' : 3,
        'device' : 'cuda'
    }
}) 
