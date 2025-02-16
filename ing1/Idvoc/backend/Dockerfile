FROM registry.zarak.fr/library/python:3.11-alpine
ENV FLASK_APP=app

ADD requirements.txt requirements.txt
RUN pip install -r requirements.txt

ADD app.py app.py

EXPOSE 6000
CMD python3 app.py; flask run --host 0.0.0.0 --port 6000
