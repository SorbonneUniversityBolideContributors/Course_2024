# Setup bolide

This file contains various information to setup the bolide.

### Table of contents

- [Table of contents](#table-of-contents)
- [Connect RPI4 to Wifi network](#connect-rpi4-to-wifi-network)
    - [AP mode](#ap-mode)
    - [Wifi mode](#wifi-mode)
- [Connect to RPI4 via SSH](#connect-to-rpi4-via-ssh)
    - [Troubleshooting](#troubleshooting)
- [Nodes to launch on the RPI4 to use the robot](#nodes-to-launch-on-the-rpi4-to-use-the-robot)
- [Connect PS4 controller to RPI4](#connect-ps4-controller-to-rpi4)
- [Tips](#tips)

## Connect RPI4 to Wifi network

Thei're two configuration for the RPI4: Access Point mode (AP) and Client mode (Wifi). If you want to do this config again on Debian 11 (bullseye) or previous release (on a new RPI4 or when you change the SD card) please follow this instructions: [https://raspberrytips.fr/point-acces-wifi-raspberry-pi/](https://raspberrytips.fr/point-acces-wifi-raspberry-pi/).
Then you can use the `wifi.sh` script to switch between the two modes by placing it in the root of the bolide (you can find the script in the `Bolide scripts` folder).

### AP mode

To activate the AP mode, use the commands bellow 
```sh
sudo ./wifi.sh AP
sudo reboot
```

In AP mode, the RPI4 is the rooter and you can connect to it with your computer.
The SSID is `bolide1_AP` (or `bolide2_AP`) and the password is on the google drive (to get all **password** for this project check the `/PFE_2024/Identifiers` file on the google drive)

Once you're connected, the IP is `192.168.42.10` (for both bolide) see the section below to connect to the RPI4 via SSH.

### Wifi mode

To activate the Wifi mode, use the commands bellow 
```sh
sudo ./wifi.sh Wifi
sudo reboot
```

In Wifi mode, the RPI4 is connected to a wifi rooter (to connect via SSH see section below).

The advantage of this mode is that the robot and your computer are connected to the internet through the rooter.

If you want to change the rooter, you need to edit the file `/etc/wpa_supplicant/wpa_supplicant.conf`:

```sh
sudo vim /etc/wpa_supplicant/wpa_supplicant.conf
```

and add the following lines:

```conf
network={
    ssid="SSID"
    psk="PASSWORD"
    priority=1
}
```

where `ssid` is the name of the rooter and `psk` is the password. You can add multiple rooter by changing the `priority` value (the higher the value, the higher the priority).

## Connect to RPI4 via SSH

- First, ensure that your computer is connected to the same network as the RPI4.

- Then, use ping to find the ip address of the RPI4 with `bolide1.local` (or `bolide2.local`).

- You can now connect to the RPI4 using 

```bash
ssh bolide2@[ip address]
```

the password is on the google drive (to get all **password** for this project check the `/PFE_2024/Identifiers` file on the google drive)

### Troubleshooting


If you can't find the RPI4 on the network and there is no network called `bolide1_AP` (or `bolide2_AP`), you can:
- connect to the RPI4 via ethernet cable and ping the RPI4 with `bolide1.local` (or `bolide2.local`).
- connect a screen and a keyboard to the RPI4.

When connected to the RPI4, you can change the network configuration as explained in the section above.

## Nodes to launch on the RPI4 to use the robot

- To make the robot operational, you need to launch the following nodes on the RPI4:

```bash
roslaunch planning_bolide ready_for_nav.launch
```

- If you only want the control node (for teleoperation for example), you can launch the following node:

```bash
rosrun control_bolide control_node.py
```

- If you only want the sensors nodes, you can use the following launch file:

```bash
# with processing
roslaunch perception_bolide perception.launch
```

```bash
# without processing
roslaunch perception_bolide main_publisher.launch
```

## Connect PS4 controller to RPI4

- Firt, in a terminal, go to the bluetoothctl command line interface:

```bash
sudo bluetoothctl
```

- Then, turn yout PS4 controller into pairing mode by pressing the PS button and the share button at the same time.
- Use the command bellow to scan for bluetooth devices:

```bash
scan on
```

- Once you've found the PS4 controller, note the MAC address and use the following commands to pair, trust and connect to the PS4 controller:

```bash
pair [MAC address]
```

```bash
trust [MAC address]
```

```bash
connect [MAC address]
```

- You can now use the PS4 controller to control the robot.

(If you need a more detailed explanation, check this [link](https://www.baeldung.com/linux/bluetooth-via-terminal))

## Tips

- Don't forget to pull the `course_2024_pkgs` repository on the RPI4 when you make changes on your computer.