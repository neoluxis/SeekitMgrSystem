import sys
import argparse

from flask import Flask, request, redirect, url_for, render_template, flash, session
from flask.globals import app_ctx

from flaskr.config import Config
from flaskr.models import db, FormalMember, Checkins
from datetime import datetime

from utils.cryptus import hash_pswd, check_pswd


class Flaskr:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config.from_object(Config)

        db.init_app(self.app)

        self.register_routes()
        self.register_hooks()

    def register_routes(self):
        self.register_usermgr_routes()

        @self.app.route('/')
        def index():
            return render_template('home.html')

    def register_hooks(self):
        @self.app.before_request
        def before_request():
            pass

    def register_usermgr_routes(self):
        @self.app.route('/signup', methods=['GET', 'POST'])
        def signup():
            return 'signup page'

        @self.app.route('/signin', methods=['GET', 'POST'])
        def signin():
            return 'signin page'

        @self.app.route('/signout', methods=['GET', 'POST'])
        def signout():
            return 'signout page'

    def clear_db(self):
        """ 清空数据库的所有数据 """
        with self.app.app_context():
            db.drop_all()
            db.create_all()
            print("Database has been cleared and reset.")

    def run(self):
        with self.app.app_context():
            db.create_all()

        self.app.run(debug=True,
                     host=self.app.config['APP_HOST'],
                     port=self.app.config['APP_PORT'])


def main(opt):
    app_instance = Flaskr()
    if opt.clear_db:
        app_instance.clear_db()
    else:
        app_instance.run()


def parse_opt():
    parser = argparse.ArgumentParser(description="Flask Application CLI")
    parser.add_argument('--clear-db', action='store_true', help='Clear and reset the database')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_opt()
    main(args)
