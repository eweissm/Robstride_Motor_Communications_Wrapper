"""
Serial Communication Interface
By Eric Weissman
4/18/2025

Description:

SerialCommunicator: Bidirectional Serial Communication Interface for Arduino

This module defines the SerialCommunicator class, which facilitates real-time,
synchronized communication between a host computer (Python) and an Arduino via
a serial port. It supports three communication modes:
    1. TwoWay - Simultaneous sending and receiving of Byte arrays.
    2. oneWayFromArduino - Only receive data from Arduino.
    3. OneWay2Arduino - Only send data to Arduino.

Key Features:
- Sync byte protocol to ensure data alignment.
- Transmission of float arrays using `struct` for byte-level control.
- Threaded architecture for non-blocking send/receive operations.
- Optional logging of communication frequency.
- Data queue for easy access to received data.
- Configurable number of bytes for transmission and reception.

Usage:
- Instantiate the class with serial port settings and desired mode.
- Use `start()` to begin communication.
- Use `set_data_to_send()` to update outgoing bytes data.
- Use `stop()` to end communication cleanly.

Example:
    communicator = SerialCommunicator(port="COM3", baudrate=115200, direction='TwoWay')
    communicator.start()
    ...
    communicator.set_data_to_send([0.1, 0.2, ..., 0.9]) # dont call this too often or you will make the CPU sad
    latest_buffer = communicator.get_buffer()
    ...
    communicator.stop()

Note:
Ensure the baudrate and COM port match the configuration on the Arduino.
"""

import serial
import struct
import threading
import time
from collections import deque
import csv
from queue import Queue
import os

class SerialCommunicator:
    def __init__(self,
                 port,
                 baudrate = 115200,
                 n_Bytes_to_arduino=9,
                 n_bytes_from_arduino=9,
                 verbose=True,
                 logFreq=False,
                 buffer_size=100,
                 CSVPath = None,
                 DesiredFreq = 10000):

       
        self.port = port  # COM port for the Arduino
        self.baudrate = baudrate # selected baud rate for serial comms. Must match arduino's
        self.n_Bytes_to_arduino = n_Bytes_to_arduino # num bytes to send to arduino
        self.n_Bytes_from_arduino = n_bytes_from_arduino # num bytes to received from the arduino
        self.sync_byte = 0xAA # byte used to sync up binary messages being sent
        
        print("Opening Port")
        numAttempts = 10
        for i in range(numAttempts):
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                print("Connected Port")
                break

            except:
                print(f"Failed to connect to Port. Attempt#{i}")

        self.buffer_size = buffer_size
        self.running = False # state of communication threads
        self.data_to_send = [0x00] * self.n_Bytes_to_arduino # message to send to the arduino
        self.verbose = verbose # option selecting to print sent and received data
        self.expected_bytes = self.n_Bytes_from_arduino # length of message from the arduino
        self.buffer = deque(maxlen=self.buffer_size)  # Constantly updating buffer which will store the received data from the arduino
        self.timeCode_buffer = deque(maxlen=self.buffer_size)  # Constantly updating buffer which will store the received data from the arduino

        self.logFreq = logFreq # option to measure communication freq
        self.data_queue = Queue()
        self.Writer_data_queue = Queue()
        self.CSVPath = CSVPath
        self.DesiredFreq = DesiredFreq
        self.log_BufferSize = 2000

        # Deques to store recent timestamps for frequency calculation
        self.send_timestamps = deque(maxlen=self.log_BufferSize)
        self.recv_timestamps = deque(maxlen=self.log_BufferSize)
        self.send_counter = 0
        self.recv_counter = 0

        self._lock = threading.Lock()
        
        self.NumTimeouts = 0

        


    def start(self): #starts communication threads according to selected direction/ protocal

        time.sleep(.1) # sleep for stability
        self.running = True


        threading.Thread(target=self.listen_thread_twoWay, daemon=True).start()
        threading.Thread(target=self.write_thread_twoWay, daemon=True).start()
            
        if self.CSVPath is not None:
            threading.Thread(target=self.data_writer, daemon=True).start()

    def get_buffer(self):  # Buffer accessor method
        return list(self.buffer)
    
    def get_TimeCodeBuffer(self):  # Buffer accessor method
        return list(self.timeCode_buffer)

    # def data_writer(self):
        
    #     buffer = []


    #     with open(self.CSVPath, 'w', newline='') as f:
    #         writer = csv.writer(f)

    #         lastTime = time.perf_counter()

    #         while self.running:

    #             now = time.perf_counter()
    #             dt = now - lastTime

    #             if dt >= 1/self.DesiredFreq:
    #                 lastTime=now
    #                 try:
    #                     data = self.data_queue.get(timeout=1)
    #                     buffer.append(data)

    #                     if len(buffer) >= self.buffer_size:
    #                         writer.writerows(buffer)
    #                         buffer.clear()
    #                 except:
    #                     continue
    #             else:
    #                 time.sleep((1/self.DesiredFreq)/10)

    def data_writer(self):
        
        buffer = []
        writer_buffer = []

        ListenerCSV = os.path.expanduser(self.CSVPath)
        WriterCSV = os.path.expanduser(self.CSVPath.replace(".csv", "_Writer.csv"))


        ListenerCSV_file = open(ListenerCSV, 'w', newline='')
        ListenerCSV_writer = csv.writer(ListenerCSV_file)

        WriterCSV_file = open(WriterCSV, 'w', newline='')
        WriterCSV_writer = csv.writer(WriterCSV_file)

        

        lastTime = time.perf_counter()

        while self.running:

            now = time.perf_counter()
            dt = now - lastTime

            if dt >= 1/self.DesiredFreq:
                lastTime=now
                try:
                    writer_buffer.append(self.Writer_data_queue.get(timeout=1))
                    buffer.append(self.data_queue.get(timeout=1))

                    if len(buffer) >= self.buffer_size:
                        ListenerCSV_writer.writerows(buffer)
                        WriterCSV_writer.writerows(writer_buffer)
                        buffer.clear()
                        writer_buffer.clear()
                except:
                    continue
            else:
                time.sleep((1/self.DesiredFreq)/10)

    def log_frequency(self, timestamps, label, counter_name):
        # helper func to record and display the communication frequency

        if self.logFreq:
            now = time.time()
            timestamps.append(now)
            counter = getattr(self, counter_name)
            counter += 1
            if len(timestamps) == timestamps.maxlen and counter >= timestamps.maxlen:
                duration = timestamps[-1] - timestamps[0]
                freq = len(timestamps) / duration if duration > 0 else 0

                if abs(freq-self.DesiredFreq) > 0.1*self.DesiredFreq:
                    print("WARNING: Serial frequencies is <10 percent off desired frequency.")


                print(f"[{label}] Avg Frequency (last {len(timestamps)}): {freq:.2f} messages/sec")
                counter = 0
                if label == "Writer":
                    print(f"[{label}]: Number of timeouts: {self.NumTimeouts}")
                    self.NumTimeouts = 0
            setattr(self, counter_name, counter)

    def _read_exact(self, n):
        """Read exactly n bytes or return None on timeout/EOF."""
        buf = bytearray()
        deadline = time.perf_counter() + self.ser.timeout if self.ser.timeout else None
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                # timed out or port closed
                return None
            buf.extend(chunk)
            # Optional hard deadline (avoids long stalls)
            if deadline and time.perf_counter() > deadline:
                return None
        return bytes(buf)

    def listen_thread_twoWay(self):
        # thread to control listening in the two way communication context.

      
        # one-time flush on start
        self.ser.reset_input_buffer()
        while self.running:
            # scan for sync byte
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != self.sync_byte:
                continue  # keep scanning

            data_bytes = self._read_exact(self.expected_bytes)
            if data_bytes is None:
                # partial frame: discard and resync
                continue

            # print(data_bytes)

            self.buffer.append(data_bytes)
            self.timeCode_buffer.append(time.time_ns()/10**9)  # seconds with nanosec precision 
            
            if self.CSVPath is not None:
                self.data_queue.put([time.time(), data_bytes.hex(',')])

            self.log_frequency(self.recv_timestamps, "Listener", "recv_counter")  # measure freq

            if self.verbose:
                print(f"[Listener] Received: {data_bytes}")

            time.sleep((1.0 / self.DesiredFreq)*.1)  # sleep to avoid overloading the cpu

    def SingleWrite(self, msg):
        #write a single message. msg is a byte array

        payload = bytearray()
        payload.append(self.sync_byte)

        for i in range(self.n_Bytes_to_arduino):
            payload.append(msg[i])
        # print(payload)
        self.ser.write(payload)  # send message to the arduino
        
        time.sleep(.01)
        
        #drain response
        self.drainSerial()

    def drainSerial(self, FirstMsg = False):
        data_bytes=[]
        while self.ser.in_waiting >= 1:  # if we have something to read
            data_bytes.append(self.ser.read(1))  # read the expected bytes
        # print(data_bytes)
        
        if data_bytes:
            # print("Drained data:", data_bytes)
            return True
        else:
            if FirstMsg:
                print("Warning: No data to drain. This could indicate a problem with the communication/ motors.")
            return False

    
    def write_thread_twoWay(self):
        interval = 1.0 / self.DesiredFreq
        next_t = time.perf_counter()
        
        # set write_timeout to keep things bounded under hiccups
        self.ser.write_timeout = 0.01

        while self.running:
            now = time.perf_counter()
            if now < next_t:
                # Always yield; tiny sleep reduces burstiness at 1 kHz
                time.sleep(min(0.5 * interval, next_t - now))
                continue

            next_t += interval

            with self._lock:
                # Take a snapshot; do NOT reuse the mutable list reference
                out = bytes(self.data_to_send)

            # Build payload once
            payload = bytes([self.sync_byte]) + out

            # Single write call minimizes USB packetization jitter
            try:
                self.ser.write(payload)

            except serial.SerialTimeoutException:
                # count it or ignore; don't print in the hot path
                self.NumTimeouts += 1
                pass
                    
            self.log_frequency(self.send_timestamps, "Writer", "send_counter")  # measure freq

            if self.CSVPath is not None:
                self.Writer_data_queue.put([time.time(), payload.hex(',')])

            # print(struct.unpack('<f', bytes(payload[-16:-12]))[0])

            if self.verbose:
                print(f"[Writer] Sent: {bytes(self.data_to_send)}")

    def GetDataQueue(self):
        with self._lock:
            if not self.data_queue.empty():
                return self.data_queue.get(timeout=1)
            else:
                return [0.0]

    def stop(self):
        self.running = False
        self.ser.close()

    def set_data_to_send(self, new_data):
        
        # with self._lock:
        #     if len(new_data) == self.n_Bytes_to_arduino:
        #         self.data_to_send = new_data
        #     else:
        #         raise ValueError(f"Expected {self.n_Bytes_to_arduino} bytes")
        with self._lock:
            self.data_to_send = new_data
        

if __name__ == "__main__":
    communicator = SerialCommunicator(port="COM14", baudrate=1_000_000, n_Bytes_to_arduino=12*3, n_bytes_from_arduino=12*3, verbose=False, logFreq=True, DesiredFreq=1000)
    communicator.start()

    try:
        while True:
            time.sleep(1)
            # Dynamically change data if needed
            EID = [0x04, 0x00, 0xFD, 0x7F]
            dataBuf = [0x00]*8
            
            communicator.set_data_to_send(EID + dataBuf)

    except KeyboardInterrupt:
        communicator.stop()
        print("Communication stopped.")