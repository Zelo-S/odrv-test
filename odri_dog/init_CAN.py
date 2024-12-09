import os

def startCAN():
    os.system("sudo ip link set can0 type can bitrate 1000000 && sudo ifconfig can0 txqueuelen 100000 && sudo ifconfig can0 up")
    os.system("sudo ip link set can1 type can bitrate 1000000 && sudo ifconfig can1 txqueuelen 100000 && sudo ifconfig can1 up")
    
def downCAN():
    os.system("sudo ifconfig can0 down")
    os.system("sudo ifconfig can1 down")

startCAN()
# downCAN()