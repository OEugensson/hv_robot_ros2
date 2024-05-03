import select
import threading
import os
import rclpy
import traceback


class listen(threading.Thread):
    # listener class for getting data from robot
    def __init__(self, q, client):
        threading.Thread.__init__(self)
        self.q = q
        self.client = client
        self.callback = None
        self.setDaemon(True)

    def run(self):
        # runs a listener thread, reading in data and placing it in a queue
        try:
            while True:
                # Wait for incoming data from the RCP or for the listener
                # pipe to close, indicating that we are
                (r, w, x) = select.select(
                    (self.client.s, self.client.listener_pipe[0]), (), ()
                )
                if self.client.listener_pipe[0] in r:
                    # listener pipe signaled the thread to exit
                    os.close(self.client.listener_pipe[0])
                    return
                if self.client.s in r:
                    (t, l, v) = self.client.receive_next_message()
                    # if not self.client.read_async_message(t,l,v):
                    # TODO: might need to change name rather than doing a method overload
                    if not self.client.do_async_message(t, l, v):
                        self.q.put((t, l, v))
        except OSError as e:
            rclpy.logerr("SOCKET ERROR")
            rclpy.logwarn(e)
            if e.errno != errno.ECONNRESET:
                raise
            client.reconnect()
        except Exception as e:
            rclpy.logerr("NON-SOCKET ERROR")
            err = traceback.format_exc()
            rclpy.logerr(e)
            rclpy.loginfo(err)
            # raise
