import numpy as np


class Character:
    """
    成员在协会的角色，分为权限等级和分组信息。每种信息占一个二进制位。
    """
    TRAINEE = 0
    FORMAL_MEMBER = 1 << 0
    VICE_PRESIDENT = 1 << 1
    PRESIDENT = 1 << 2
    GRADUATED = 1 << 3

    SUBMARINE = 1 << 4
    QUOD_COPTER = 1 << 5
    POWER = 1 << 6
    METER = 1 << 7


if __name__ == '__main__':
    print(Character.VICE_PRESIDENT | Character.FORMAL_MEMBER | Character.SUBMARINE)
