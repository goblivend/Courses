# pylint: disable=E0401,C0116,C0114,W0602,C0103,W0702

from flask import Flask, jsonify, request
import redis
import requests

app = Flask(__name__)


@app.route("/")
def index():
    for res, url in [
        ("visit_counter", "http://backend:6000/get_and_inc/visit_counter"),
        ("text", "http://backend:6000/lorem"),
            ]:
        try:
            r = requests.get(url, timeout=1)
            assert r.ok
            globals()[res] = r.text
        except Exception as e:
            print(e, flush=True)
            globals()[res] = "backend problem. Unable to get content. Check logs"
    return (
        f"""
    <!doctype html>
    <html>
    <head><title>Welcome to the awesome app !</title></head>
    <body><h1>Welcome to the awesome app</h1>
    <p>Number of visits: {visit_counter}</p><br/>
    <p>{text}</p>
    </body>
    </html>
    """,
        200,
    )


@app.route("/test")
def test():
    redis_reachable = True
    try:
        redis_co = redis.Redis(host="redis", port="6379")
        redis_co.ping()
    except:
        redis_reachable = False
    print(str(dict(request.headers)), flush=True)
    with_nginx = request.headers.get("X_NGINX", "false").lower() == "true"
    try:
        r = requests.get("http://backend:6000", timeout=0.2)
        back_reachable = r.ok
    except:
        back_reachable = False
    return (
        jsonify(
            {
                "accessed_with_nginx": with_nginx,
                "redis_unreachable": not redis_reachable,
                "back_reachable": back_reachable,
            }
        ),
        200,
    )
