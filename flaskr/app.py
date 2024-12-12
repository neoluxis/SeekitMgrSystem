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
        self.register_api()

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

    def register_api(self):
        @self.app.route('/api/account/signup', methods=['POST'])
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

        @self.app.route('/api/account/signin', methods=['POST'])
        def api_signin():
            # info_submitted = request.json
            # username = info_submitted.get('username')
            # password = info_submitted.get('password')
            # user = FormalMember.query.filter_by(username=username).first()
            # if user is None:
            #     return jsonify({'status': 'failed', 'message': 'User not found'})
            # if not check_pswd(password, user.register_date, user.password):
            #     return jsonify({'status': 'failed', 'message': 'Password incorrect'})
            if not self.check_passwd(request.json.get('username'), request.json.get('password')):
                return jsonify({'status': 'failed', 'message': 'Password incorrect'})
            return jsonify({'status': 'success', 'message': 'Login success'})

        @self.app.route('/api/account/remove', methods=['POST'])
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

        @self.app.route('/api/checkin/list', methods=['POST'])
        def api_checkin_list():
            is_admin = self.check_admin(request.json.get('username'))
            if is_admin:
                checkins = Checkins.query.all()
            else:
                checkins = Checkins.query.filter_by(username=request.json.get('username')).all()
            return jsonify({'status': 'success', 'checkins': [c.to_dict() for c in checkins]})

        @self.app.route('/api/checkin', methods=['POST'])
        def api_checkin():
            ck_date = datetime.now().strftime('%Y-%m-%d')
            ck_time = datetime.now().strftime('%H:%M:%S')
            ck_rec = Checkins(
                username=request.json.get('username'),
                chdate=ck_date,
                chtime=ck_time,
            )
            db.session.add(ck_rec)
            db.session.commit()
            self.get_logger().info(f"User {request.json.get('username')} checkin at {ck_date}/{ck_time}")
            return jsonify(
                {'status': 'success', 'message': f"User {request.json.get('username')} checkin at {ck_date}/{ck_time}"})

        @self.app.route('/api/credit/stat', methods=['POST'])
        def api_credit_stat():
            """
            统计成员的积分，包括各种统计
            :return:
            """
            return jsonify({'status': 'success', 'message': 'Credit stat success'})

        @self.app.route('/api/credit/update', methods=['POST'])
        def api_credit_update():
            """
            更新成员的积分
            :return:
            """
            return jsonify({'status': 'success', 'message': 'Credit update success'})

        @self.app.route('/api/account/clear_trainee', methods=['POST'])
        def api_clear_trainee():
            if not self.check_passwd(request.json.get('username'), request.json.get('password')):
                return jsonify({'status': 'failed', 'message': 'Password incorrect'})
            user = FormalMember.query.filter_by(username=request.json.get('username')).first()
            if (user.character & (Character.VICE_PRESIDENT | Character.PRESIDENT)) == 0:
                return jsonify({'status': 'failed', 'message': 'Permission denied'})
            self.clear_trainee()
            return jsonify({'status': 'success', 'message': 'All accounts of trainees have been removed'})

    def clear_db(self):
        """ 清空数据库的所有数据 """
        with self.app.app_context():
            db.drop_all()
            db.create_all()
        self.get_logger().info("Database has been cleared.")

    def clear_trainee(self):
        """ 清空数据库中的所有新生 """
        with self.app.app_context():
            trainees = FormalMember.query.filter_by(character=Character.TRAINEE).all()
            for trainee in trainees:
                db.session.delete(trainee)
            db.session.commit()
            self.get_logger().info("All trainees have been removed.")

    def delete_table_by_name(self, table_name):
        """ 删除数据库中的指定表 """
        with self.app.app_context():
            if table_name in db.metadata.tables.keys():
                db.metadata.tables[table_name].drop(db.engine)
                self.get_logger().info(f"Table {table_name} has been removed.")
            else:
                self.get_logger().error(f"Table {table_name} not found.")

    def create_table_by_name(self, table_name):
        """ 创建数据库中的指定表 """
        with self.app.app_context():
            try:
                db.metadata.tables[table_name].create(db.engine)
                self.get_logger().info(f"Table {table_name} has been created.")
            except Exception as e:
                self.get_logger().error(f"Failed to create table {table_name}.")
                self.get_logger().error(e)

    def run(self):
        with self.app.app_context():
            db.create_all()
        self.app.run(debug=True,
                     host=self.app.config['APP_HOST'],
                     port=self.app.config['APP_PORT'])

    @staticmethod
    def check_passwd(username, password):
        user = FormalMember.query.filter_by(username=username).first()
        if user is None:
            return False
        return check_pswd(password, user.register_date, user.password)

    @staticmethod
    def check_admin(username):
        user = FormalMember.query.filter_by(username=username).first()
        if user is None:
            return False
        return (user.character & (Character.VICE_PRESIDENT | Character.PRESIDENT)) != 0


def main(opt):
    app_instance = Flaskr()
    if opt.clear_db:
        app_instance.clear_db()
    elif opt.clean_trainee:
        app_instance.clear_trainee()
    elif opt.delete_table != '':
        app_instance.delete_table_by_name(opt.delete_table)
    elif opt.create_table != '':
        app_instance.create_table_by_name(opt.create_table)
    else:
        app_instance.run()


def parse_opt():
    parser = argparse.ArgumentParser(description="Flask Application CLI")
    parser.add_argument('--clear-db', action='store_true', help='Clear and reset the database')
    parser.add_argument('--clean-trainee', action='store_true', help='Remove all trainees')
    parser.add_argument('--delete-table', type=str, default='', help='Delete a specific table')
    parser.add_argument('--create-table', type=str, default='', help='Create a specific table')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_opt()
    main(args)
