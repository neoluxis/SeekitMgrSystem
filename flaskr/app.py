from flask import Flask, request, redirect, url_for
from flask import render_template, flash, session
from config import Config
from models import db, FormalMember, Checkins
from datetime import datetime
import hashlib

app = Flask(__name__)
app.config.from_object(Config)
db.init_app(app)


def hash_pswd(password, date):
    """
    生成密码，将注册日期与密码拼接后进行md5加密，这样即使密码相同，也会生成不同的加密串
    :param password: 明文密码
    :param date: 注册日期
    :return: 加密后的密码
    """
    ps = date + password
    return Config.hasher(ps.encode()).hexdigest()


@app.route('/')
def index():
    return 'Hello, World!'


def main():
    with app.app_context():
        db.create_all()
    app.run(debug=True, host=app.config['APP_HOST'], port=app.config['APP_PORT'])


if __name__ == '__main__':
    main()
