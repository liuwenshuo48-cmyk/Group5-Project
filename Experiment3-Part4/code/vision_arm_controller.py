import socket
import subprocess


HOST="0.0.0.0"
PORT=5005


ALLOW=[
    "G1",
    "G2",
    "G3",
    "G4",
    "G5",
    "G6"
]


server=socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)


server.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)


server.bind((HOST,PORT))

server.listen(1)


print("等待视觉连接...")


conn,addr=server.accept()


print("视觉连接:",addr)


busy=False


while True:

    data=conn.recv(1024)


    if not data:
        break


    msg=data.decode().strip()


    print("收到:",msg)


    if msg not in ALLOW:

        print("忽略:",msg)

        continue



    if busy:

        print("机械臂忙，忽略:",msg)

        continue



    busy=True


    print("执行抓取:",msg)



    subprocess.run(
        [
            "python3",
            "pick_from_grid.py",
            msg
        ]
    )



    print("动作完成:",msg)



    # ===== 新增 =====
    # 告诉Jetson当前目标完成

    conn.send(
        b"DONE"
    )


    busy=False



conn.close()
