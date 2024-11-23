import can

with can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000) as can_back:
    can_back.flush_tx_buffer()