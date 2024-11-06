from flaskr.config import Config


def hash_pswd(password, date):
    """
    生成密码，将注册日期与密码拼接后进行md5加密，这样即使密码相同，也会生成不同的加密串
    :param password: 明文密码
    :param date: 注册日期
    :return: 加密后的密码
    """
    ps = date + password
    return Config.hasher(ps.encode()).hexdigest()


def check_pswd(password, date, hash_pswd):
    """
    检查密码是否正确
    :param password:  明文密码
    :param date: 注册日期
    :param hash_pswd: 加密后的密码
    :return:
    """
    return hash_pswd == hash_pswd(password, date)


if __name__ == '__main__':
    print(hash_pswd('123456', '2024-01-01'))
