import os
import hashlib
import socket


def get_local_ip():
    # 获取本机 IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.254.254.254', 1))  # 连接到一个外部IP来获取本机的局域网IP
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = '127.0.0.1'  # 备用IP，若连接失败则为本地回环
    finally:
        s.close()
    return local_ip


def am_i_on_cloud():
    my_ip = get_local_ip()
    if my_ip.startswith('31.'):
        return True
    return False


class Config:
    # 应用开放的主机和端口
    APP_HOST = '0.0.0.0'
    APP_PORT = 5999

    SECRET_KEY = 'ilovecandy'

    # 本地开发使用的数据库配置
    SQLALCHEMY_DATABASE_URI = 'mysql+pymysql://cieusche:qiushi2024@localhost/CieuscheMgr'

    # 服务器上使用的数据库配置
    if am_i_on_cloud():
        SQLALCHEMY_DATABASE_URI = 'mysql+pymysql://m11154_cieusche:Qiushi2024@mysql8.serv00.com/m11154_CieuscheMgr'

    # 禁用追踪数据库的修改
    SQLALCHEMY_TRACK_MODIFICATIONS = False

    # 用于密码加密的哈希算法
    hasher = hashlib.md5
