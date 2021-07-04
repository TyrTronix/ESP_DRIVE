# ESP_Drive

A light-weight asyncronous Arduino Libary for the ESP_Drive to control 1-4 motors alone, or with optical encoder feedback.
Ultrasonic distance functionality is also built-in.

**Library Features:**<br/>
- Individual motor control rotate clockwise, counter clockwise, stop, brake<br/>
- Collective motor control drive forward, backward, left, right, stop, brake<br/>
- Encoder tick counter via pin interrupts
- Encoder velocity calculations performed via timers in ticks/second. (From the diameter of your wheels the velocity in mm/s can be calculated)
- Distance measurement via HC-SR04
- Speed controller, tries to maintain speed at the requested setting using encoder feedback.

**Board Specs:**<br/>
- Input power 6-12V 2S Lion Batteries Recommended<br/>
- 2 x DRV8833 dual motor drivers<br/>
- 3.3V and 5V regulators<br/>
- Reverse Voltage protection on main input<br/>
- Automatic program upload through USB-C port<br/>
- ESP32 reset and boot push buttons<br/>
- 2 x Optical encoder sensors for perferated disk encoders found on common acrylic robot chassis<br/>
- HC-SR04 4 pin connector for Distance measurements<br/>
- 4 Pin I2C pin connector for additional sensor<br/>
- All remaining GPIO are broken out via 2.54mm pitch male headers<br/>



All of my repositories on this account are open sourced through the MIT license and are free to use by anyone for any purpose.<br/>
If you would like to donate/tip me, then Nano is a quick and easy way to do that.<br/>
Visit https://nano.org/ for instructions on how to create a wallet and buy/collect some Nano - [WeNano](https://wenano.net/) is a great app to get started<br/>
Once you have a wallet with some Nano all you need to do is scan the QR code below and donate :)<br/>

<p align="center">
  <img src="https://user-images.githubusercontent.com/71822838/124380430-d6c30700-dcbc-11eb-9252-5cee4706d862.png"/>
</p>



