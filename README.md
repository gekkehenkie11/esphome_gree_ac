# Open source WIFI module replacement for Gree protocol based AC's for Home Assistant.
This repository adds support for ESP-based WiFi modules to interface with Gree/Sinclair AC units.
It's forked from https://github.com/piotrva/esphome_gree_ac, big thanks to @piotrva for his work!

My fork currently differs from the original code in the following ways. What I did:

1) Fixed the fan mode, tested on Gree/Daizuki/TGM AC's.
2) Fixed the dropping of commands
3) Fixed the rejection of commands
4) Fixed reporting of current temp
5) Fixed the Fahrenheit mode
6) Implemented an optional silent mode (no beeping), only works for module sent commands (not for
   remote control sent commands)
   
It's now compatible with GRJWB04-J / Cs532ae wifi modules

# Current state:
No known problems! if you run into an issue though, please let me know.

# HOW TO 
You can flash this to an ESP module. I used an ESP01-M module, like this one:
https://nl.aliexpress.com/item/1005008528226032.html
So that’s both an ESP01 and the ‘adapter board’ for 3.3V ↔ 5V conversion (since the ESP01 uses 3.3V and the AC uses 5V).


Create a new project in the ESPbuilder from Home assistant and use my YAML (from the examples directory), copy or modify the info from the generated YAML to mine, where it says '[insert yours]'.
Then you should be able to compile it. I think you can flash directly from Home Assistant,
but I downloaded the compiled binary and flashed with: https://github.com/esphome/esphome-flasher/releases

See the 4 module cable photos for wiring (for flashing and the wiring for connecting it to your AC). The connector is a 4 pins “JST XARP-04V”, you can for example order them here: https://es.aliexpress.com/item/1005009830663057.html. Alternatively it's possible to just use dupont cables and then put tape around the 4 ends to simulate the form of a connector (so that it makes it thicker), so that it will sit still in the connector socket, but make sure all 4 cables make connection, I tried that first and you might need to make adjustments because of 1 cable not making solid connection (this might result in errors in the log). So best to use the real connector. 


After you've connected the module to your AC, it should pop under settings/integrations/esphome as a 'new device' and then you can add it to HA. If not, check if it started a WIFI access point, which it will do if it can't connect to your home wifi. You can then connect to that and configure it from there (via 192.168.4.1)

**USE AT YOUR OWN RISK!**
