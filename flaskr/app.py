import sys
import argparse
import logging

from flask import Flask, request, redirect, url_for
from flask import render_template, flash, session, jsonify
from flaskr.config import Config
from flaskr.models import db, FormalMember, Checkins
from datetime import datetime

from utils.cryptus import hash_pswd, check_pswd
from Consts import Character


class Flaskr:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config.from_object(Config)

        db.init_app(self.app)

        self.register_routes()
        self.register_hooks()
        self.register_api_routes()

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(self.app.config['LOG_LEVEL'])
        self.logger.addHandler(logging.StreamHandler(sys.stdout))

    def get_logger(self):
        return self.logger

    def register_routes(self):
        self.register_usermgr_routes()

        @self.app.route('/')
        def index():
            # return render_template('home.html')
            return 'Index Page'

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

    def register_api_routes(self):
        @self.app.route('/api/signup', methods=['POST'])
        def api_signup():
            info_submitted = request.json
            username = info_submitted.get('username')
            name = info_submitted.get('name')
            school_id = info_submitted.get('school_id')
            phone = info_submitted.get('phone')
            password = info_submitted.get('password')
            date = datetime.now().strftime('%Y-%m-%d')
            hashpsw = hash_pswd(password, date)
            user_character = Character.TRAINEE
            user = FormalMember(username=username, name=name, school_id=school_id, password=hashpsw,
                                register_date=date, phone=phone, character=user_character)
            all_usernames = [u.username for u in FormalMember.query.all()]
            if username in all_usernames:
                return jsonify({'status': 'failed', 'message': 'Username already exists'})
            try:
                db.session.add(user)
                db.session.commit()
                self.get_logger().info(f"User {username} has been registered.")
            except Exception as e:
                self.get_logger().error(f"Failed to register user {username}.")
                self.get_logger().error(e)
                return jsonify({'status': 'failed', 'message': 'Failed to register user'})
            return jsonify({'status': 'success', 'message': 'User registered successfully'})

        @self.app.route('/api/signin', methods=['POST'])
        def api_signin():
            info_submitted = request.json
            username = info_submitted.get('username')
            password = info_submitted.get('password')
            user = FormalMember.query.filter_by(username=username).first()
            if user is None:
                return jsonify({'status': 'failed', 'message': 'User not found'})
            if not check_pswd(password, user.register_date, user.password):
                return jsonify({'status': 'failed', 'message': 'Password incorrect'})
            return jsonify({'status': 'success', 'message': 'Login success'})

        @self.app.route('/api/remove', methods=['POST'])
        def api_remove():
            info_submitted = request.json
            username = info_submitted.get('username')
            password = info_submitted.get('password')
            user = FormalMember.query.filter_by(username=username).first()
            if user is None:
                return jsonify({'status': 'failed', 'message': 'User not found'})
            if not check_pswd(password, user.register_date, user.password):
                return jsonify({'status': 'failed', 'message': 'Password incorrect'})
            try:
                db.session.delete(user)
                db.session.commit()
                self.get_logger().info(f"User {username} has been removed.")
            except Exception as e:
                self.get_logger().error(f"Failed to remove user {username}.")
                self.get_logger().error(e)
                return jsonify({'status': 'failed', 'message': 'Failed to remove user'})
            return jsonify({'status': 'success', 'message': 'User removed successfully'})

        @self.app.route('/api/checkin', methods=['POST'])
        def api_checkin():
            pass

    def clear_db(self):
        """ 清空数据库的所有数据 """
        with self.app.app_context():
            db.drop_all()
            db.create_all()
            print("Database has been cleared and reset.")

    def clear_trainee(self):
        """ 清空数据库中的所有新生 """
        with self.app.app_context():
            trainees = FormalMember.query.filter_by(character=Character.TRAINEE).all()
            for trainee in trainees:
                db.session.delete(trainee)
            db.session.commit()
            print("All trainees have been removed.")

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
    elif opt.clean_trainee:
        app_instance.clear_trainee()
    else:
        app_instance.run()


def parse_opt():
    parser = argparse.ArgumentParser(description="Flask Application CLI")
    parser.add_argument('--clear-db', action='store_true', help='Clear and reset the database')
    parser.add_argument('--clean-trainee', action='store_true', help='Remove all trainees')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_opt()
    main(args)
