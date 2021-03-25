# MDP-Algorithm
Raymond Hartono, Ziang Wu

## Algorithms  

run robot
```
run runner/MazeRunner.run()
```

run simulator
```
run simulator/Simulator.run()
```

switch from fastest path to exploration
```java
private static boolean explorationMode = true;
```

enable extra exploration
```java
private static boolean extraMode = false;
```


## RPi
connect rpi
```
ssh pi@192.168.4.4
2021rpiGrp4

cd Desktop/CommMods/
sudo python3 multiProcessMod.py
```

reset communication
```
ps -fA | grep python3
sudo kill -9 $(ps -A | grep python3 | awk '{print $1}')
```