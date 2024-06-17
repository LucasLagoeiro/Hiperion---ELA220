from PyQt5.QtCore import *
import time

class ThreadClass(QThread):
    signal = pyqtSignal(int)

    def __init__(self):
        super(ThreadClass, self).__init__(None)
        self.index=0
        self.is_running = True
    
    def run(self):
        print("Thread start")
        cnt = 0
        while True:
            cnt += 1
            if cnt == 99 : cnt=0
            time.sleep(0.01)
            self.signal.emit(cnt)

    def stop(self):
        self.is_running = False
        self.terminate()

    
    