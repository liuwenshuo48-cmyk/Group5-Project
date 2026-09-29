#!/usr/bin/env python3


import socket
import threading
import rclpy


from rclpy.node import Node
from vision_msgs.msg import Detection2DArray



class GridCheck(Node):


    def __init__(self):

        super().__init__(
            "real_grid_check"
        )


        self.subscription = self.create_subscription(
            Detection2DArray,
            "/detections",
            self.callback,
            10
        )


        self.required_stable_frames=8


        self.last_targets=None

        self.stable_count=0


        self.task_started=False


        self.task_queue=[]


        self.current_target=None


        self.completed_targets=set()



        self.client=socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )


        self.client.connect(
            (
                "10.42.0.89",
                5005
            )
        )


        self.get_logger().info(
            "TCP connected to arm controller"
        )


        # 启动反馈监听
        threading.Thread(
            target=self.receive_done,
            daemon=True
        ).start()



        self.get_logger().info(
            "Multi target planner started"
        )



    # ==========================
    # 接收机械臂完成信号
    # ==========================

    def receive_done(self):


        while True:


            try:


                data=self.client.recv(
                    1024
                )


                if not data:
                    break



                msg=data.decode().strip()



                if msg=="DONE":


                    self.get_logger().info(
                        "收到机械臂完成反馈"
                    )


                    if self.current_target:

                        self.completed_targets.add(
                            self.current_target
                        )


                    self.send_next()



            except Exception as e:


                self.get_logger().error(
                    str(e)
                )

                break





    def bbox_to_grid(self,x,y):


        if x<140 or x>677 or y<29 or y>587:

            return None



        if y<308:


            if x<315:

                return 1


            elif x<494:

                return 2


            else:

                return 3



        else:


            if x<315:

                return 4


            elif x<494:

                return 5


            else:

                return 6






    def detect_targets(self,msg):


        targets=set()



        for det in msg.detections:


            if not det.results:

                continue



            x=det.bbox.center.position.x

            y=det.bbox.center.position.y



            grid=self.bbox_to_grid(
                x,y
            )



            if grid:


                targets.add(
                    f"G{grid}"
                )



        return targets






    def start_task(self,targets):


        order=[

            "G1",
            "G2",
            "G3",
            "G6",
            "G5",
            "G4"

        ]



        self.task_queue=[

            x for x in order

            if x in targets

        ]



        self.get_logger().info(
            f"task order:{self.task_queue}"
        )


        self.task_started=True


        self.send_next()






    def send_next(self):


        while self.task_queue:


            target=self.task_queue.pop(0)



            if target in self.completed_targets:

                continue



            self.current_target=target



            self.client.send(
                target.encode()
            )


            self.get_logger().info(
                f"send:{target}"
            )


            return





        self.get_logger().info(
            "all targets completed"
        )







    def callback(self,msg):


        targets=self.detect_targets(
            msg
        )



        if not targets:

            self.last_targets=None

            self.stable_count=0

            return



        if self.task_started:

            return



        if targets==self.last_targets:


            self.stable_count+=1


        else:


            self.last_targets=targets

            self.stable_count=1




        if self.stable_count>=self.required_stable_frames:


            self.get_logger().info(
                f"detected:{list(targets)}"
            )


            self.start_task(
                targets
            )






def main():


    rclpy.init()


    node=GridCheck()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    finally:


        node.destroy_node()


        rclpy.shutdown()





if __name__=="__main__":

    main()
