# CRSF-Converter

This project uses an Arduino to read in the CRSF protocol and output the PPM protocol.
It will also read in the LTM (Lightweight Telemetry) protocol and send telemetry data back through CRSF.

I created this so I could use an ExpressLRS receiver with an old Naze32 flight controller.

**If this project helped you in any way, please consider supporting my work through a [PayPal donation](paypal.me/jacobgunderson).**


## Hardware Setup

I used an Arduino Pro Mini but most Arduino boards should work.

#### Arduino - CRSF Receiver
Vcc - 5v (to power the receiver)  
Gnd - Gnd  
Pin 0 (TX) - RX  
Pin 1 (RX) - TX  

#### Arduino - Flight Controller
Vcc - 5v (to power the Arduino and the receiver)  
Gnd - Gnd  
Pin 10 - PPM input pin  
Pin 11 - LTM output pin  


## Software Setup

The Arduino communicates at 115200 with the CRSF receiver.  
&nbsp;&nbsp;&nbsp;&nbsp;(For an ExpressLRS receiver, change the RCVR_UART_BAUD parameter to 115200.)

The Arduino communicates at 9600 baud with the flight controller to receive LTM (telemetry).  
&nbsp;&nbsp;&nbsp;&nbsp;(For Betaflight or iNav, on the "Ports" page, enable LTM telemetry at 9600 baud.)


## Other Notes

Currently, the only supported telemetry parameters are:
* Battery voltage
* Latitude
* Longitude
* Ground speed
* Heading
* Altitude
* GPS sat

For some reason when the heading is displayed on my radio, it's divided by 10.  
&nbsp;&nbsp;&nbsp;&nbsp;For example, a heading of 125º is displayed as 12.5º.
