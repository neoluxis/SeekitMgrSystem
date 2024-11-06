from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()


class FormalMember(db.Model):
    __tablename__ = 'formal_members'
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(128), unique=True, nullable=False)  # 系统用户名
    name = db.Column(db.String(64), unique=False, nullable=False)  # 真实姓名
    school_id = db.Column(db.String(32), unique=True, nullable=False)  # 学号
    password = db.Column(db.String(64), unique=False, nullable=False)  # 登入系统的密码
    register_date = db.Column(db.Date, unique=False, nullable=False)  # 账户注册日期
    character = db.Column(db.Integer, unique=False, nullable=False, default=0)  # 权限
    phone = db.Column(db.String(11), unique=True, nullable=False)  # 手机号


class Checkins(db.Model):
    __tablename__ = 'checkins'
    id = db.Column(db.Integer, primary_key=True)
