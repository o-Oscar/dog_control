import threading
from queue import Queue


def background_input(out_q: Queue):
    while True:
        inp = input()
        out_q.put(inp)


class AsyncInput:
    def __init__(self):
        self.last_msg = None
        self.input_queue = Queue()

        thread = threading.Thread(target=background_input, args=(self.input_queue,))
        thread.daemon = True
        thread.start()

    def check_for_msg(self):
        while self.input_queue.qsize() > 0:
            self.last_msg = self.input_queue.get()
        return self.last_msg
