from flask import Flask, make_response , request, jsonify
app = Flask(__name__)
from duckieenv import DuckieEnv

real_env  = DuckieEnv()

@app.route('/step', methods=['GET', 'POST'])
def step():
    d = request.args.get("action")
    if not d:
        return make_response(jsonify({"err": "action is invalid, Got None!"}),404)
    action = int(d)
    depth_array, reward, terminate, info = real_env.step(action)
    info["tof_range"] = info["tof_range"].tolist()
    data = {'depth_array': depth_array.tolist(), 'reward': reward, 'terminate': terminate, 'info': info}
    jsonified = None
    try:
        jsonified = jsonify(data) 
    except:
        real_env.stop_bot()
    
    return  make_response(jsonified, 200)


@app.route('/reset', methods=['GET'])
def reset():
    obs = real_env.reset()
    data = {'observation' : obs.tolist()}
    return  make_response(jsonify(data), 200)


if __name__ == '__main__':
   app.run(host='0.0.0.0', debug = True)