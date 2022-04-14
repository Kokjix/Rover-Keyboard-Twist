# Rover-Keyboard-Twist
Bu ros paketi rover'ın alt yürürünün klavye yardımı ile sürülmesi için yapılmıştır.
## Paketin kullanılması

## Not: Gerçekte denemeden önce 22 rover ve 21 rover için twist değerleni değiştirin 22 sisteminde lineer hızların mutlak değeri 2.0 angular hızların mutlak değeri 1.0 olarak iyidir. 21 sistemi ve simülasyonlar için lineer hızlar 0.2 angular hızlar 0.1 iyidir.

Workspace'e aşağıdaki şekilde klonelanır.
```
git clone https://github.com/Kokjix/Rover-Keyboard-Twist.git
```
Ardından workspace buildlenir.
```
catkin build
```
Ve sourcelanır.
```
source devel/setup.bash
```
Ardından şu paketler çalıştırılır.
```
roslaunch rover_20_serial rover_serial.launch
```
```
rosrun rover_20_control rover_msg_creator.py
```
```
rosrun rover_21_control roverodom.py
```
### Launch dosyaları
21 Rover Simülasyon için:
```
roslaunch rover_keyboard_twist keyboard_21_sim.launch
```
21 Rover gerçek alt yürür sürüş için:
```
roslaunch rover_keyboard_twist keyboard_21.launch
```
22 Rover Simülasyon için:
```
roslaunch rover_keyboard_twist keyboard_22.launch
```

21 rover simülasyonu için "/cmd_vel" topic publishlenir. 21 rover gerçek alt yürür için "/key_teleop/cmd_vel" topic publishlenir. 22 rover simülasyonu için "/drive_system/twist".
