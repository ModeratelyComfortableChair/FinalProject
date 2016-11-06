This zip contains two projects: one is the server, which you run on your laptop
and the other is an example showing how to talk to the server from the EV3.
As provided to you, the EV3 code is a basic example program that prints the data received 
from the server to the screen.  It is up to you to get the provided code (the wifi package)
working properly with the rest of your software.

To get it working, in Elicpse go to File->Import->General->Projects from Folder or Archive
Select this zip file using the "Archive" button
Ensure the two entries labeled as "Eclipse project" are selected; you can deselect the entry
labeled as "Wifi Code Fall 2016.zip_expanded"
Click Finish

When it's done importing, right click on the EV3WiFiClient project and 
select leJOS EV3->Convert to leJOS project

You can now run the WiFi project as an EV3 project on your robot
and the DPMServer project as a regular Java program.

You will need to configure the robot to connect to the WiFi network, in addition to connecting
with your laptop:

SSID: DPM
Password:dddpppmmm

You can have your EV3 connect via the menu (WiFi) or via the EV3Control program.  Remember to 
plug in the USB WiFi adapter you were given at the start of the semester - Bluetooth is the only wireless
protocol built into the EV3 brick.

Once that's all set up, you will need to configure your robot to connect to your laptop.
To do this, first get the IP address you were assigned by the DPM router.  

On Windows 10, you can do this by clicking on "Properties" on the DPM network once you are connected to it.
The resulting setting page will have your IP address listed at the botton as "IPv4 adress".

On Linux, the ifconfig utility will show it under "inet addr" for a network likely to be named
something along the lines of "wlan0".  Your GUI of choice (GNOME, KDE, etc.) will also display
this information somehow.

Regardless of your OS, there are plenty of online guides that will show you how to do this
if you have trouble.

Once you have your IP address, you need to modify the string SERVER_IP in WifiTest.java to match.
By default, it is set to 192.168.2.6 but your laptop is almost guaranteed to be assigned something else.
You will also need to modify the TEAM_NUMBER variable as appropriate.

Once that's done, run the DPMServer program as a normal Java program on your computer.

Once the DPMServer program is running, launch the EV3WiFiClient program on your robot; it should connect
to your laptop and wait for data.  You can now enter numbers into the DPMServer GUI and click "start".  
If all goes well, your EV3 should now display the data you just entered.

The WiFi test code uses System.out.println statements that print to both the screen and, if connected,
the EV3Control console.  This is particularly useful if you need to debug as reading output
is much easier on your laptop than on the LCD screen.  

When you integrate the WiFi package into your code, note that printing from the WiFi code itself can be disabled by setting debugPrint to false in the WifiConnection constructor if so desired.

If there are any questions or bug reports, please post on the discussion board or e-mail me at
michael.smith6@mail.mcgill.ca