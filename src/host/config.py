from easydict import EasyDict as edict

BOT_NAME = 'bishoybot'
cfg = edict({
    'camera_topic' : f"/{BOT_NAME}/camera_node/image/compressed",
    'wheels_topic' : f"/{BOT_NAME}/wheels_driver_node/wheels_cmd",
    'tof_topic' : f"/{BOT_NAME}/front_center_tof_driver_node/range",
    'raw_depth_topic': '/raw_depth',
    'camera_param_res' : [f"/{BOT_NAME}/camera_node/res_w", f"/{BOT_NAME}/camera_node/res_h"],
    'camera_width': 640,
    'camera_height': 480,
    'bot_speed': 0.3,
    'max_bot_delta_speed' : 0.25,
    'discourage_rate' : 0.70,
    'rotation_speed' : 0.4,
    'reverse_speed' : 0.5,
    'tof_min_range': 0.0,
    'tof_max_range': 1.0,
    'tof_min_range_threshold' : 0.20,
    'depth_dim' : 224,
    'block_size': 1,
    'num_discrete_actions' : 5, # should always be odd!
    'dtype' : 'float32',
    'training': {
        'dqn': {
            'reward_threshold' : 30000,
            'seed' : 0,
            'eps_test': 0.05,
            'eps_train' : 0.1,
            'buffer_size' : 20000,
            'lr' : 2e-4,
            'gamma' : 0.99,
            'n_step' : 3,
            'target_update_freq' : 320,
            'epoch' : 100,
            'step_per_epoch' : 10000,
            'step_per_collect' : 10,
            'update_per_step' : 0.125,
            'batch_size' : 64,
            'hidden_sizes' : [64, 64],
            'training_num' : 10,
            'test_num' : 3,
            'logdir' : 'log',
            'alpha' : 0.6,
            'beta' : 0.4,
            'device' : 'cuda',
            'resume' : True
        }
    }
}) 
