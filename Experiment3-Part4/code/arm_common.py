#!/usr/bin/env python3

# 真机小工具公共部分
# pymycobot 3.6.3 + MechArm270
# 返回码 -1 不代表失败，一律以读回角度为准

import time
import sys

from pymycobot import MechArm270


def connect():

    mc = MechArm270(
        "/dev/ttyAMA0",
        1000000
    )

    time.sleep(0.8)

    return mc



def read6(fn, tries=5):

    for _ in range(tries):

        v = fn()

        if isinstance(v,(list,tuple)) and len(v)==6:
            return list(v)

        time.sleep(0.4)

    return None



def show(mc,tag):

    print(
        "%s 角度: %s"
        %
        (
            tag,
            read6(mc.get_angles)
        )
    )


    print(
        "%s 坐标: %s"
        %
        (
            tag,
            read6(mc.get_coords)
        )
    )



# =============================
# 自定义机械臂限位
# =============================

LIMIT_MIN = [
    -165,
    -90,
    -180,
    -165,
    -115,
    -175
]


LIMIT_MAX = [
    165,
    135,
    70,
    165,
    115,
    175
]



def send_angles_direct(mc,angles,speed):

    from pymycobot.common import ProtocolCode

    ints=[
        mc._angle2int(a)
        for a in angles
    ]

    return mc._mesg(
        ProtocolCode.SEND_ANGLES,
        ints,
        speed,
        has_reply=True
    )



def goto(
    mc,
    target,
    speed,
    timeout=20,
    tol=1.5
):


    for k,(v,lo,hi) in enumerate(
        zip(
            target,
            LIMIT_MIN,
            LIMIT_MAX
        )
    ):

        if not lo <= v <= hi:

            mc.stop()

            sys.exit(
                "!!! 目标 J%d 超出限位"
                %
                (k+1)
            )



    print(
        "-> 目标:",
        target,
        "速度:",
        speed
    )



    send_angles_direct(
        mc,
        target,
        speed
    )



    t0=time.time()


    while time.time()-t0 < timeout:


        a=read6(
            mc.get_angles
        )


        if a:

            err=max(
                abs(x-y)
                for x,y in zip(a,target)
            )


            if err < tol:
                break


        time.sleep(0.2)



    time.sleep(0.5)



    a=read6(
        mc.get_angles
    )


    err=[

        round(
            abs(x-y),
            2
        )

        for x,y in zip(a,target)

    ] if a else None



    print(
        "到位角度:",
        a,
        "(用时 %.1fs)"
        %
        (time.time()-t0)
    )


    print(
        "各关节残差:",
        err,
        "最大:",
        max(err) if err else None
    )


    return a,err



# =============================
# 示教 / 抓取
# =============================


import json
import os


POINTS_FILE="/home/er/taught_points.json"



def soft(mc):

    for j in range(1,6):

        mc.release_servo(j)

        time.sleep(0.1)



def hold(mc):

    for j in range(1,7):

        mc.focus_servo(j)

        time.sleep(0.1)

    time.sleep(0.3)



# ===== 原始夹爪函数 =====

def gripper(mc,state,name=""):

    """
    state:
    0 开
    1 合
    """

    r = mc.set_gripper_state(
        state,
        70
    )


    time.sleep(2)


    try:

        v = mc.get_gripper_value()

    except Exception:

        v=None



    print(
        "  夹爪%s 返回:%s 读值:%s"
        %
        (
            name,
            r,
            v
        )
    )

    return v



def teach_point(name,tip):


    mc=connect()


    show(
        mc,
        "起始"
    )


    gripper(
        mc,
        0,
        "开"
    )


    if input(
        "[1] 即将变软：回车继续 "
    ).strip()=="q":

        sys.exit(0)



    soft(mc)



    print(
        "已变软，可以手动摆位"
    )


    if input(
        "[2] 回车记录"
    ).strip()=="q":

        hold(mc)

        sys.exit(0)



    a=read6(
        mc.get_angles
    )


    pts={}


    if os.path.exists(
        POINTS_FILE
    ):

        pts=json.load(
            open(
                POINTS_FILE
            )
        )



    pts[name]=a



    json.dump(
        pts,
        open(
            POINTS_FILE,
            "w"
        ),
        indent=1
    )


    print(
        name,
        "=",
        a
    )



    input(
        "[3] 恢复力矩回车"
    )


    hold(mc)



def load_points(*names):


    if not os.path.exists(
        POINTS_FILE
    ):

        sys.exit(
            "没有points文件"
        )


    pts=json.load(
        open(
            POINTS_FILE
        )
    )


    missing=[

        x for x in names
        if x not in pts

    ]


    if missing:

        sys.exit(
            "缺少:"+str(missing)
        )


    return pts



def lifted(pt,deg):


    up=list(pt)

    up[1]-=deg

    return up
