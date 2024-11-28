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
    username = db.Column(db.String(64), db.ForeignKey('formal_members.username'))  # 签到人
    chdate = db.Column(db.Date, unique=False, nullable=False)  # 签到日期
    chtime = db.Column(db.Time, unique=False, nullable=False)  # 签到时间

    longitude = db.Column(db.Float, unique=False, nullable=True, default=0.0)  # 经度
    latitude = db.Column(db.Float, unique=False, nullable=True, default=0.0)  # 纬度
    location = db.Column(db.String(128), unique=False, nullable=True, default='Unknown')  # 签到地点

    def to_dict(self):
        ret = {
            'username': self.username,
            'name': '',
            'chdate': str(self.chdate),
            'chtime': str(self.chtime),
            'longitude': self.longitude,
            'latitude': self.latitude,
            'location': self.location,
        }
        user = FormalMember.query.filter_by(username=self.username).first()
        ret['name'] = user.name
        return ret
