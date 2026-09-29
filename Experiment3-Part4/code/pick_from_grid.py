#!/usr/bin/env python3

import sys
import time
from arm_common import connect, goto, gripper, show


ZERO=[
0,0,0,0,0,0
]


POINTS={


# =====================
# 口香糖盒 G1-G3
# =====================

"G1_HIGH":[
-28.03,
39.37,
4.21,
-9.14,
-7.47,
-0.35
],

"G1_PICK":[
-33.04,
52.73,
3.51,
1.05,
-4.65,
0.61
],


"G2_HIGH":[
-1.05,
45.7,
-16.96,
-0.79,
-31.2,
0.17
],

"G2_PICK":[
-1.84,
52.73,
3.51,
1.05,
-4.65,
0.61
],


"G3_HIGH":[
21.0,
24.34,
27.77,
4.21,
15.99,
-0.17
],

"G3_PICK":[
21.09,
33.31,
27.86,
6.59,
19.86,
-0.26
],



# =====================
# 胶带 G4-G6
# =====================


"G4_HIGH":[
-21.44,
65.03,
-0.43,
-0.96,
45.08,
0.26
],

"G4_PICK":[
-19.24,
72.42,
-0.26,
-3.69,
40.34,
0.26
],


"G5_HIGH":[
-2.19,
63.28,
-0.7,
0.08,
31.37,
-0.35
],

"G5_PICK":[
-2.72,
69.34,
-0.17,
-3.16,
32.78,
-0.26
],


"G6_HIGH":[
19.51,
72.94,
-8.08,
8.26,
41.3,
-0.52
],

"G6_PICK":[
19.33,
73.56,
-2.81,
-3.6,
41.13,
0.35
],



# =====================
# 放置点
# =====================


"GUM_PLACE":[
-73,
51.59,
0.17,
-9.14,
-7.47,
-0.35
],


"TAPE_HIGH":[
52.38,
71.63,
-3.25,
-3.07,
46.66,
-0.35
],


"TAPE_PLACE":[
48.6,
77.6,
-3.95,
-1.4,
46.4,
-0.17
]

}




def pick(grid):


    if grid not in [
        "G1",
        "G2",
        "G3",
        "G4",
        "G5",
        "G6"
    ]:

        print("暂不支持:",grid)
        return



    mc=connect()


    high=POINTS[grid+"_HIGH"]
    pick=POINTS[grid+"_PICK"]



    show(
        mc,
        "开始"
    )



    print("回零")

    goto(
        mc,
        ZERO,
        15
    )


    # =========================
    # 每次抓取前先打开夹爪
    # =========================

    print("预打开夹爪")

    gripper(
        mc,
        0,
        "开"
    )



    print(grid,"HIGH")

    goto(
        mc,
        high,
        15
    )



    print(grid,"PICK")

    goto(
        mc,
        pick,
        15
    )



    # =========================
    # 关闭夹爪
    # =========================


    print("夹爪关闭")

    gripper(
        mc,
        1,
        "合"
    )


    time.sleep(1)



    # =========================
    # 抬升
    # =========================


    print("返回HIGH")


    goto(
        mc,
        high,
        15
    )



    # =========================
    # 分类放置
    # =========================


    if grid in [
        "G1",
        "G2",
        "G3"
    ]:


        print(
            "移动口香糖放置点"
        )


        goto(
            mc,
            POINTS["GUM_PLACE"],
            15
        )


    else:


        print(
            "移动胶带放置区域"
        )


        print(
            "TAPE_HIGH"
        )


        goto(
            mc,
            POINTS["TAPE_HIGH"],
            15
        )


        print(
            "TAPE_PLACE"
        )


        goto(
            mc,
            POINTS["TAPE_PLACE"],
            15
        )



    print("释放")


    gripper(
        mc,
        0,
        "开"
    )


    time.sleep(2)



    print("回零")


    goto(
        mc,
        ZERO,
        15
    )


    show(
        mc,
        "结束"
    )


if __name__=="__main__":


    if len(sys.argv)<2:

        print(
            "用法: python3 pick_from_grid.py G1-G6"
        )

        sys.exit()



    pick(
        sys.argv[1]
    )
