# Robstride_Motor_Communications_Wrapper
Code to communicate and control Robstride Motors

This post will explain how to operate the RobStride motors with a minimal python code.



# Prerequisite Knowledge

This guide will assume you have prior knowledge on

- Coding: Python and C++
- How to upload code to Teensy 4.1
- Basic circuit assembly


# Required Materials

- Teensy 4.1
- [Can Transceiver](https://www.amazon.com/WWZMDiB-TJA1050-Controller-Interface-Module/dp/B0C7ZBDG4B)
- [Robstride Motor ](https://robstride.com/)(any model)
- [CAN-USB bridge](https://www.aliexpress.us/item/3256807756256932.html?gps-id=pcStoreLeaderboard&scm=1007.22922.271278.0&scm_id=1007.22922.271278.0&scm-url=1007.22922.271278.0&pvid=f40ca26c-f76c-4a96-8849-f68cb6855bdb&_t=gps-id%3ApcStoreLeaderboard%2Cscm-url%3A1007.22922.271278.0%2Cpvid%3Af40ca26c-f76c-4a96-8849-f68cb6855bdb%2Ctpp_buckets%3A668%232846%238116%232002&pdp_ext_f=%7B%22order%22%3A%2249%22%2C%22eval%22%3A%221%22%2C%22sceneId%22%3A%2212922%22%2C%22fromPage%22%3A%22recommend%22%7D&pdp_npi=6%40dis%21USD%2135.72%2135.72%21%21%21247.86%21247.86%21%4021033d1217680045541867315ed117%2112000046389474672%21rec%21US%21%21ABXZ%211%210%21n_tag%3A-29910%3Bd%3A506073d6%3Bm03_new_user%3A-29895&spm=a2g0o.store_pc_home.smartLeaderboard_2008097548487.1005007942571684&gatewayAdapt=glo2usa)
- A computer with the Arduino IDE and a python IDE installed
- A voltage supply (voltage depends on the motor you pick)
    - NOTE: the motors often give a voltage range (e.g., 24-60V) and then give a design voltage (typ. 48V). Avoid running the motor near the edge of the voltage ranged (i.e., don't run the motor at 24V)



WARNING: (Pinching Hazard) The Robstride motors are strong, can move very quickly, and may move unexpectedly. While the motor is operating or even powered, you should avoid handling the motor.

WARNING: Improperly setting up the circuits in this guide can irreparably damage the motor, teensy, or even your computer. Double-check your work and keep your wiring clean.


# Using Robstride's Motor Setup Tool

We will start by finding the motor's CAN ID.

1.  To do this, download the RobStride motor studio and the required driver:

[https://github.com/RobStride/MotorStudio/releases](https://github.com/RobStride/MotorStudio/releases)

2.  Set up the circuit:

![](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/preimages/kkklppzAmAuvoh4MnLHhDHs1.png?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

Refer to your specific motor's data sheet for the correct voltage and wiring on the motor: [https://github.com/RobStride/Product_Information/tree/main/Product%20Literature](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature)

3.  Launch the motor tool

You will see the following screen:

![](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/preimages/7Ayoz9JpePvgx5S9QM8x654B.png?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

4.  Power the motor

5.  Plug in the USB

6.  Click "Refresh COM"

A COM device should connect

7. Click "Open COM"

8.  Click "Detection Device"

In the box above the button, a CAN device will populate saying "CAN: {some number} id: {some id string}"

9. The number after "CAN" is your CAN ID in decimal. Convert your ID to hex: [https://www.binaryhexconverter.com/decimal-to-hex-converter](https://www.binaryhexconverter.com/decimal-to-hex-converter)

You can change the CAN ID with the "set ID" button

You can test the motor motion by clicking and holding "JOG+"

# Overview of Control

![](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/preimages/FdqpGdd5MFmAYtZ9tWqjJT5m.png?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)





# Minimal Example

This method allows you to directly control RS motors through a python script.



## Circuit

![](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/preimages/AvLC4SEq6MpvyLKG5LXIGs7H.png?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

Refer to your specific motor's data sheet for the correct voltage and wiring on the motor: [https://github.com/RobStride/Product_Information/tree/main/Product%20Literature](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature)

Make sure to double-check your wiring and to keep everything clean to avoid shorting! Mistakes in your circuit can damage the motor or Teensy beyond repair.

## Code

### Teensy Code

Configure your Arduino IDE for teensey: [https://www.pjrc.com/teensy/tutorial.html](https://www.pjrc.com/teensy/tutorial.html)

In the code, update:`numMotors`, `motorIds`, and `Can1_Ids` (same as motorIds): Teensey Code\Q1_CDE_MCU_SerialCANRelay_v4.ino


 This script:
1. reads serial over USB from host computer. This message is formatted as [sync byte], [motor 1 CAN msg], [motor 2 CAN msg], ...
2. parses serial message into individual motor CAN msgs
3. send each motor the CAN msg
4. Send each motor a msg asking for its current angle
5. When each motor responds, send over serial to host computer the current angle of each motor


```
// vvvvvvvvvvvvvv UPDATE ME  vvvvvvvvvvvv
const uint16_t numMotors = 1;
const byte motorIds[numMotors] = { 0x0A };
const byte Can1_Ids[1] = { 0x0A };//assign motors to specific can buses based on wiring
```


### Python Code

WARNING: the robstride class was designed and tested in a linux ROS environment. For quick and dirty applications, the class will work, however I recommend using the ros2 deployment which is much more robust!

Setting up your environment:

For conda:
```
conda create -n RobStrideControl python=3.10.12
conda activate RobStrideControl
pip install numpy==2.2.6
pip install pyserial==3.5
```


To run the code, you must (1) configure the config/config.py file, (2) run the scripts/simple_test_motion.py file. simple_test_motion.py will drive the motor to track a sinusoid.

 

# Running the Motor Checklist

- [ ] Get CAN ID of the Motor
- [ ] Assemble the circuit and double-check your wiring
- [ ] Update Teensy Code variables: `numMotors`, `motorIds`, and `Can1_Ids`
- [ ] Upload Teensy Code
- [ ] Update config file
- [ ] Plug in the motor
- [ ] Run simple_test_motion.py

# Motor Parameters

Found on the Discord (not in the data sheet)

| Motor | Reflected inertia |
| --- | --- |
| 00 | .001kg/m^2 |
| 03 | .02kg/m^2 |
| 04 | .04kg/m^2 |

# 

# Common Issues



**Communications**: If you are struggling to communicate with a motor, there can be several causes:

1. Wiring: the majority of you issues will be from bad connections.
    1.  Verify your motor has the required voltage
    1. Use a multimeter to check the connection of the CAN wires
1. Correct CAN connections
    1. Ensure CAN-H is connected to CAN-H and  CAN-L is connected to CAN-L
1. Incorrect CAN ID



**Jittery Motion**: The motor motion can become jittery. There are several potential causes

1. Inconsistent communications. Depending on your OS, the microcontroller, and the number of motors on a single CAN bus, you may find communications are unstable. Try to reduce the coms frequency to see if communication rates stabilize. 
    1. It is recommended to use a linux system running a real time kernel to control the motors
    1. Ensure USB port is not being overwhelmed. multiple devices on the same USB port can corrupt motor state messages and increase latency
1. Poorly tuned motor gains. Noisy velocity or position gains and overly stiff controller gains can cause jitters



**Motor Shuts Down**

Internal faults in the motor may trigger the motor to shut down after startup. This can be due to an overtemperature fault. **Check the 0x02 reply frame to read exact fault.**

Ensure:

- Running at 48V
    - If you run near 24V, then large current spikes can cause a voltage drop below 24V, causing the motor to shut down
- Not over temp





# Old Files [Ignore these unless you are Eric]



[RobstrideMotorMessagesClass_V5.py](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/GxX1QSGAxnGVEGrevImx3f7S.py?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[MotorSerialCommunicator.py](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/yLLOg2PkPLEz3fP9cwaEKZeA.py?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[config_v1.py](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/Q2tbhp6dRAXOzDcD6oJ6PNku.py?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)



[Q1_CDE_MCU_FullSQuadSerialCANRelay_v3.ino](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/AnoDYWAqvW0XQzhPySqMSNAN.ino?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[Q1_CDE_MCU_SQuadSerialCANRelay_v2.ino](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/dMiU49EXzhhWsdWYl8jnSLWU.ino?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[Q1_CDE_MCU_FullSQuadSerialCANRelay_v1.ino](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/r7BnBJthXGlWYSGFN9F1z7kv.ino?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[RobstrideMotorMessagesClass_V4.py](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/phXIItkPYLJ8VKIzx4YZ2TQd.py?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)

[MotorSerialCommunicator.py](https://slabstatic.com/prod/assets/2p0nnwtd/post/8g1ux1kn/attachments/h0o0PbA5abut2TIOY3c83Nae.py?jwt=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJodHRwczovL3NsYWJzdGF0aWMuY29tIiwiZXhwIjoxNzg5Nzc0MzE3LCJpYXQiOjE3ODg1NjQ3MTcsImlzcyI6Imh0dHBzOi8vYXBwLnNsYWIuY29tIiwianRpIjoiMzM5NDQ4azBuNDNsdjU2ZGFlcWQxaWYxIiwibmJmIjoxNzg4NTY0NzE3LCJwYXRoIjoicHJvZC9hc3NldHMvMnAwbm53dGQvcG9zdC84ZzF1eDFrbiJ9.XtXzQqC5YiypbOqRisHH58SJBF4kkApdsCb5lB7t33g0tNVMWt-CEPVNxbPFJV0AaqCFJKFPi9NwyfcPb3hmINYUJKWujP0-pW1vDZ4_m39_EidsziErz2hqNpe9Db6f2UNz3uTBqVdYpbi0ErklNgBt_FaNeyi0xX9q3VIt7du6aeB4DmuuFvTWynaCpZV-IUw5I0O6_DOWNRjOe9Viaprx5tlJT7AgEP_FkloVLvO7P5lSOzPxDRXi75Z1yo2DaKwGRF39XyUMmJk69c9owOCc4bWt6v79i2A-HOPyHOiLniHJLrXPAbPlLZhPazRy8DBOUZDQ1YCUkrBQpXvtTA)
