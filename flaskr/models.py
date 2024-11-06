from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()


class FormalMember(db.Model):
    __tablename__ = 'formal_members'
    id = db.Column(db.Integer, primary_key=True)


class Checkins(db.Model):
    __tablename__ = 'checkins'
    id = db.Column(db.Integer, primary_key=True)
