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
