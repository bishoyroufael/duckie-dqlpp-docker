from flask import Flask, make_response , request, jsonify
app = Flask(__name__)
from duckieenv import DuckieEnv

real_env  = DuckieEnv()

@app.route('/step', methods=['GET'])
def step():
    depth_array, reward, terminate, info = real_env.step()
    data = {'depth_array': depth_array.tolist(), 'reward': reward, 'terminate': terminate, 'info': info}
    return  make_response(jsonify(data), 200)


@app.route('/reset', methods=['GET'])
def reset():
    obs = real_env.reset()
    data = {'observation' : obs.tolist()}
    return  make_response(jsonify(data), 200)


if __name__ == '__main__':
   app.run(host='0.0.0.0', debug = True)