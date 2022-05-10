from flask import Flask, make_response , request, jsonify, send_file
app = Flask(__name__)
from duckieenv import DuckieEnv
import io
import numpy as np 

real_env  = DuckieEnv()

@app.route('/step', methods=['GET', 'POST'])
def step():
    d = request.args.get("action")
    if not d:
        return make_response(jsonify({"err": "action is invalid, Got None!"}),404)
    action = int(d)
    depth_array, reward, terminate, info = real_env.step(action)
    buf = io.BytesIO() 
    np.savez_compressed(buf, depth_array, np.array([reward]), np.array([terminate]), info["tof_range"])
    buf.seek(0)

    return send_file(buf, attachment_filename="data.png", mimetype='image/png')

    # Old , SLOW
    # info["tof_range"] = info["tof_range"].tolist()

    # data = {'depth_array':depth_array.tolist(), 'reward': reward, 'terminate': terminate, 'info': info}
    # jsonified = None
    # try:
    #     jsonified = jsonify(data) 
    # except:
    #     real_env.stop_bot()
    # # print("stepping") 
    # return  make_response(jsonified, 200)


@app.route('/reset', methods=['GET'])
def reset():
    obs = real_env.reset()
    buf = io.BytesIO() 
    np.savez_compressed(buf, obs)
    buf.seek(0)

    return send_file(buf, attachment_filename="data.png", mimetype='image/png')
    
    
    # data = {'observation' : obs.tolist()}
    # return  make_response(jsonify(data), 200)


if __name__ == '__main__':
   app.run(host='0.0.0.0', port = 5000, debug = True)