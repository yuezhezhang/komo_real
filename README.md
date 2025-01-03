# komo_real

## bimanual planning

change `franka.cpp` and `FrankaGripper.cpp` to 
```
const char *frankaIpAddresses[2] = {"192.168.2.55","10.10.10.10"};
const char *gripperIpAddresses[2] = {"192.168.2.55","10.10.10.10"};
```
and set `bot.cpp`
```
rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "both");
```

## llm planning

change `franka.cpp` and `FrankaGripper.cpp` to 
```
const char *frankaIpAddresses[2] = {"10.10.10.10", "192.168.2.55"};
const char *gripperIpAddresses[2] = {"10.10.10.10", "192.168.2.55"};
```
and set `bot.cpp`
```
rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "left");
```