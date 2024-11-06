import os
import hashlib
from utils.findme import am_i_on_cloud


class Config:
    # 应用开放的主机和端口
    APP_HOST = '0.0.0.0'
    APP_PORT = 5000

    SECRET_KEY = 'ilovecandy'

    # 本地开发使用的数据库配置
    SQLALCHEMY_DATABASE_URI = 'mysql+pymysql://cieusche:qiushi2024@localhost/CieuscheMgr'

    # 服务器上使用的数据库配置
    if am_i_on_cloud():
        print('I am on cloud.')
        APP_PORT = 4917
        SQLALCHEMY_DATABASE_URI = 'mysql+pymysql://m11154_cieusche:Qiushi2024@mysql8.serv00.com/m11154_CieuscheMgr'

    # 禁用追踪数据库的修改
    SQLALCHEMY_TRACK_MODIFICATIONS = False

    # 用于密码加密的哈希算法
    hasher = hashlib.md5
