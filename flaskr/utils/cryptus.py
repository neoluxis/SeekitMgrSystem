from flaskr.config import Config


def hash_pswd(password, date):
    """
    生成密码，将注册日期与密码拼接后进行md5加密，这样即使密码相同，也会生成不同的加密串
    :param password: 明文密码
    :param date: 注册日期
    :return: 加密后的密码
    """
    if type(date) is not str:
        date = str(date)
    ps = password + date
    return Config.hasher(ps.encode()).hexdigest()


def check_pswd(password, date, hash_psw):
    """
    检查密码是否正确
    :param password:  明文密码
    :param date: 注册日期
    :param hash_pswd: 加密后的密码
    :return:
    """
    return hash_psw == hash_pswd(password, date)


if __name__ == '__main__':
    print(hash_pswd('2201400216', '2024-11-07'))
