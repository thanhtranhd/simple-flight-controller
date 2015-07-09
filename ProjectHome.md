This is is a free & open source project to build a flight controller for quadro-copter flying machine. New flying control for other multirotor like tri-copter, hex, etc, will be added later on.


Features:<br>
<ul><li>Gyro dampen stabilizing quad-copter<br>
</li><li>Remote gyro gain setting via a receiver channel<br>
</li><li>Arm & disarm the controller using left stick of your radio transmitter (mode 2)<br>
</li><li>Ability to save some basic stats during flight into flash and can be retrieved later.<br>
</li><li>Command line interface for set up and stats viewing<br>
</li><li>1-micro second resolution for reading receiver signal and sending signal to ESC's<br>
</li><li>200Hz control loop (can be increased)<br>
</li><li>Work with any radio receiver/transmitter and doesn't expect any channel sequence / order<br>
</li><li>Can use the built-in CC2500 of the TI EZ430RF board, so no RX is needed in this case.<br>
</li><li>Use hardware PWM and hardware event capture.<br>
</li><li>Very low jitter<br>
</li><li>Easy to add in feature<br>
</li><li>Inexpensive: $20 for the main board. $10 for the gyro board.<br>
</li><li>Quick to build.<br>
</li><li>Very light weight and can be built very small.<br>
</li><li>Very low power consumption<br>
</li><li>Currently works in quadro-copter mode only.</li></ul>

Near future plan:<br>
<ul><li>Tricopter<br>
</li><li>Adding GPS<br>
</li><li>Accelerometer</li></ul>


<br>
<br>
It's built around a MSP430-RF2500 experimental kit available from TI webstore, a Wii Motion Plus gyro board, and some off-the-shelves speed controllers and brushless motors.<br>
<br>
<br>
The code is written in C for MSP430F2274 and is GPL V2 licensed. Used with Code Composure Studio (CCS V4).<br>
<br>
<br>

Videos:<br>
<br>
<a href='http://www.youtube.com/watch?v=7FqNSNsSOtY'>http://www.youtube.com/watch?v=7FqNSNsSOtY</a>
<br>
<a href='http://www.youtube.com/watch?v=68t_vRlQ3LA'>http://www.youtube.com/watch?v=68t_vRlQ3LA</a>

<a href='http://www.youtube.com/watch?feature=player_embedded&v=7FqNSNsSOtY' target='_blank'><img src='http://img.youtube.com/vi/7FqNSNsSOtY/0.jpg' width='425' height=344 /></a><br>
<a href='http://www.youtube.com/watch?feature=player_embedded&v=68t_vRlQ3LA' target='_blank'><img src='http://img.youtube.com/vi/68t_vRlQ3LA/0.jpg' width='425' height=344 /></a><br>
<br>
<br>
Disclaimer:<br>
This code comes with absolute no warranty. Please use it at YOUR OWN risk.<br>
<br>
<br>
Build log: <a href='http://www.rcgroups.com/forums/showthread.php?t=1335765'>http://www.rcgroups.com/forums/showthread.php?t=1335765</a>

