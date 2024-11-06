import os
import hashlib


class Config:
    APP_HOST = '0.0.0.0'
    APP_PORT = 5999

    SECRET_KEY = 'ilovecandy'
    # SQLALCHEMY_DATABASE_URI = 'sqlite:///cieuschemgr.db'
    SQLALCHEMY_DATABASE_URI = 'mysql+pymysql://cieusche:qiushi2024@localhost/CieuscheMgr'
    SQLALCHEMY_TRACK_MODIFICATIONS = False

    hasher = hashlib.md5
