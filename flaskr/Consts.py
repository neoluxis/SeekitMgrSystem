import numpy as np


class Character:
    """
    成员在协会的角色，分为权限等级和分组信息。每种信息占一个二进制位。
    """
    TRAINEE = 0  # 0000 0000 新生
    FORMAL_MEMBER = 1 << 0  # 0000 0001 正式成员
    # VICE_PRESIDENT = 1 << 1  # 0000 0010  副会长
    # PRESIDENT = 1 << 2  # 0000 0100 会长
    # GRADUATED = 1 << 3  # 0000 1000 毕业学长
    DEAN = 1 << 1  # 0000 0010 教导主任负责管理新生培训
    VICE_PRESIDENT = 1 << 2  # 0000 0010 副会长
    PRESIDENT = 1 << 3  # 0000 1000 会长

    SUBMARINE = 1 << 4  # 0001 000 水下组
    QUOD_COPTER = 1 << 5  # 0010 0000 四轴组
    POWER = 1 << 6  # 0100 0000 电源组
    METER = 1 << 7  # 1000 0000 测量组


if __name__ == '__main__':
    print(Character.VICE_PRESIDENT | Character.FORMAL_MEMBER | Character.SUBMARINE)
