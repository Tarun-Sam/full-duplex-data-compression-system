\# Full-Duplex Data Compression System



This project is a Full-duplex embedded compression and communication system using Arduino, Python GUI, serial communication, and an OLED display.



The system reads real-time analog sensor data from a potentiometer, compresses the data on the Arduino side, sends the compressed packet to a Python Tkinter GUI, validates and decompresses the packet, and can send the same compressed payload back to the Arduino. The Arduino then validates, decompresses, and displays the returned data on an OLED display.



\## Features



\- Real-time potentiometer data acquisition using Arduino

\- 50 ADC samples collected per packet

\- Deadband filtering to reduce small signal variations

\- Delta encoding for sensor value changes

\- Bit-packing and Zero-RLE based compression

\- Checksum-protected serial packet transmission

\- Python Tkinter GUI for packet monitoring

\- ADC value display in the GUI

\- Voltage value display in the GUI

\- Live voltage waveform visualization

\- Manual return and auto-return modes

\- Arduino receives returned compressed packet

\- OLED display shows returned/decompressed data



\## System Flow



1\. Potentiometer gives analog input to Arduino.

2\. Arduino reads the input using `analogRead()`.

3\. Arduino collects 50 ADC samples.

4\. The data is filtered and compressed.

5\. A compressed packet is sent to the Python GUI through serial communication.

6\. Python validates the checksum.

7\. Python decompresses the packet and displays ADC values, voltage values, and waveform.

8\. Python can send the compressed payload back to Arduino.

9\. Arduino validates and decompresses the returned packet.

10\. Arduino displays the returned data on the OLED display.



\## Folder Structure



```text

data\_compression\_system/

│

├── firmware/

│   └── data\_compression\_arduino/

│       └── data\_compression\_arduino.ino

│

├── host/

│   └── data\_compression\_gui.py

│

├── docs/

│

├── images/

│   └── project\_workflow.png

│

├── README.md

├── LICENSE

└── .gitignore

```



\## Hardware Used



\- Arduino Uno / Arduino Mega 2560

\- Potentiometer

\- OLED display

\- USB cable for serial communication

\- Jumper wires



\## Software Used



\- Arduino IDE

\- Python

\- Tkinter

\- PySerial

\- Git and GitHub



\## Communication Details



The project uses serial communication between Arduino and Python.



```text

Baud rate: 115200

```



The Arduino sends compressed packets to Python. The Python GUI validates the packet using checksum, decompresses the data, displays it, and can return the same compressed payload back to Arduino.



\## Packet Concept



The system uses a packet-based communication format.



Arduino sends a compressed packet to Python. The packet contains:



\- Start byte

\- Compression mode

\- Base ADC value

\- Data length

\- Compressed data

\- Checksum

\- End byte



The checksum is used to check whether the received packet is valid or corrupted.



\## Compression Methods Used



This project uses two compression approaches:



\### 1. Bit-Packing



Bit-packing is used when the potentiometer values are changing frequently.



Instead of sending full ADC values every time, the Arduino sends smaller delta values packed into fewer bytes.



\### 2. Zero-RLE



Zero-RLE is used when the signal is stable or repeated.



If the potentiometer value does not change much, repeated values can be represented using run-length encoding.



\## Key Idea



The system sends compressed ADC values, not voltage values.



Voltage is calculated after decompression using:



```text

Voltage = ADC Value × 5.0 / 1023

```



This keeps the transmitted packet smaller while still allowing voltage values to be shown in the Python GUI and OLED display.



\## Python GUI Features



The Python Tkinter GUI displays:



\- Received compressed packet in HEX format

\- Reconstructed ADC values

\- Voltage values

\- Live voltage waveform

\- Packet statistics

\- Compression mode

\- System log

\- Checksum status

\- Manual return and auto-return controls



\## Full-Duplex Operation



This project is full-duplex because communication happens in both directions:



```text

Arduino → Python GUI

Python GUI → Arduino

```



The Arduino sends compressed sensor data to Python. Python can then send the same compressed payload back to the Arduino either manually or automatically.



\## Status



This project is currently under development.



\## Future Improvements



\- Add more sensor inputs

\- Improve compression mode selection

\- Add more detailed packet analysis

\- Add better error handling

\- Improve OLED display layout

\- Add screenshots and demo video

