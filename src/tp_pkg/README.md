### Exercise 1

## Launch

# Default
Count limit is 100, count trigger is 50 and timer period is 200 ms.

```bash
~/ros2_ws# ros2 launch tp_pkg tp2_e1.launch.py
```

# Custom
```bash
~/ros2_ws# ros2 launch tp_pkg tp2_e1.launch.py count_limit:=20 timer_period:=0.5 count_trigger:=3
```
