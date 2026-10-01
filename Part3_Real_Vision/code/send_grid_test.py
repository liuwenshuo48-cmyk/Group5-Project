import socket
import time


HOST="10.42.0.89"
PORT=5005


client=socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)


client.connect((HOST,PORT))


while True:

    msg="G1"

    client.send(
        msg.encode()
    )

    print("发送:",msg)

    time.sleep(2)
