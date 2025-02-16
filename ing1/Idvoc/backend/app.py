# pylint: disable=E0401,C0116,C0114,W0602,C0103,W0702

import os
from flask import Flask
import redis
import lorem

redis = redis.Redis(
         host= os.environ.get("REDIS_HOST", 'redis'),
         port= '6379')

app = Flask(__name__)

@app.route("/")
def index():
    return "backend", 200

@app.route("/status")
def status():
    return "ok"

@app.route("/set/<key>/<value>", methods=['POST'])
def api_set(key, value):
    redis.set(key, value)
    return "set"

@app.route("/get/<key>")
def api_get(key):
    value = redis.get(key)
    return value

@app.route("/get_and_inc/<key>")
def api_get_and_inc(key):
    try:
        value = int(redis.get(key)) + 1
    except:
        redis.set(key, 1)
        return "1"
    redis.set(key, value)
    return str(value)

@app.route("/lorem")
def api_get_lorem():
    try:
        with open('/cache/lorem', 'r') as f:
            return f.read()
    except:
        text = lorem.text()
        with open('/cache/lorem', 'w') as f:
            f.write(text)
        return text
