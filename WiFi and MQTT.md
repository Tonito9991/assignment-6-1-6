# WiFi and MQTT
The following is a `cheat sheet` to configure WiFi and MQTT.

## WiFi
The WiFi connection with an access point is configured with the Shell on the board.
Use SEGGER RTT or the VCOM (Virtual COM, 115200/8N1) connection:Make sure that after a command a new-line (`\n`) or `\r\n` is sent.

- `help` lists all available commands. Use `wifi help` for a subgroup.
- `status` or `wifi status` shows the settings.

Help:
```
wifi                      ; Group of WiFi application commands
  help|status             ; Print help or status information
  enable|disable          ; Enable or disable WiFi connection
  set ssid "<ssid>"       ; Set the SSID
  set pwd "<password>"    ; Set the password
  set hostname "<name>"   ; Set the hostname
```
Use for example the `wifi set ssid "mySSID"` to set your AP SSID. Note that the settings get stored in the FLASH memory of the device.

Example:
```
wifi set ssid "mySSID"
wifi set pwd "This-is-my-secret-password-key"
wifi set hostname "tsm-sensor"
```
Restart the device aftwards.

## MQTT Broker on Raspberry Pi
Below the steps to install and configure `Mosquitto` on a Raspberry Pi (Debian based):
```
$ sudo apt update && sudo apt upgrade
$ sudo apt install -y mosquitto mosquitto-clients
```
Check that broker is running:
```
$ ps -aux | grep mosquito
```

Set username and password:
```
$ sudo mosquitto_passwd -c /etc/mosquitto/passwd <YOUR_USERNAME>
```
and enter your password. User name and password are needed later for the MQTT client connection below.

Edit the configuration file:
```
$ sudo nano /etc/mosquitto/mosquitto.conf
```
Add the following line to the beginning of the file:
```
per_listener_settings true
```
Add the following lines to the end of the file:
```
allow_anonymous false
listener 1883
password_file /etc/mosquitto/passwd
```
After that, reboot your system:
```
$ sudo reboot
```


## MQTT Client
In a similar way, the MQTT client can be configured and used:

`mqttclient help` gives:
```
mqttclient                ; Group of mqttclient commands
  help|status             ; Print help or status information
  log on|off              ; Turn logging on or off
  publish on|off          ; Publishing on or off
  connect|disconnect      ; Connect or disconnect from server
  set broker "<broker>"   ; Set broker name
  set id "<id>"           ; Set client ID
  set user "<user>"       ; Set client user name
  set pass "<password>"   ; Set client password
  pub "<topic>" "<txt>"   ; Publish text to a topic
```
The client only supports connections to a broker with a user name and password. So you have to configure the broker (IP or name), user and password, plus an ID which should be unique for each client.

Example:
```
mqttclient set broker "192.168.4.1"
mqttclient set id "pico"
mqttclient set user "picowsensor"
mqttclient set pass "myPicoPwd"
```

Restart the device after making the settings.

