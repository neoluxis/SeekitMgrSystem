from flask import Flask, request, redirect, url_for
from flask import render_template, flash, session
from config import Config
from models import db, FormalMember, Checkins
from datetime import datetime
import hashlib

from utils.cryptus import hash_pswd, check_pswd

app = Flask(__name__)
app.config.from_object(Config)
db.init_app(app)


@app.route('/')
def index():
    return 'Hello, World!'


def main():
    with app.app_context():
        db.create_all()
    app.run(debug=True, host=app.config['APP_HOST'], port=app.config['APP_PORT'])


if __name__ == '__main__':
    main()
