# Flask simple api to test image publishing over the web
from flask import Flask, make_response, Response
import cv2

app = Flask(__name__)
app.debug = True

@app.route('/')
def main():
    img = cv2.imread('testimg.png')
    retval, buffer= cv2.imencode('.png', img)
    return Response(b'--frame\r\n' b'Content-Type: image/png\r\n\r\n' + buffer.tobytes() + b'\r\n\r\n', mimetype='multipart/x-mixed-replace; boundary=frame') 

if __name__ == '__main__':
    app.run(host='0.0.0.0')
~                                                                                                                                                                   
~                             
