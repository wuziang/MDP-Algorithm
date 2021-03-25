# MDP-Algorithm
Raymond Hartono, Ziang Wu

Start RPi
```
ssh pi@192.168.4.4
2021rpiGrp4

cd Desktop/CommMods/
sudo python3 multiProcessMod.py
```

Reset RPi
```
ps -fA | grep python3
sudo kill -9 $(ps -A | grep python3 | awk '{print $1}')
```

Run Robot
```
run MazeRunner.run()
```

Run Simulator
```
run Simulator.run()
```