#тестовое задание 

Робот в симуляторе Gazebo автономно проезжает из точки A в точку B с объездом статических препятствий. Маршрут задаётся GPS-waypoints (WGS84). Используются лидар, depth-камера (аналог Orbbec Astra) и GPS.

- Видео демонстрации:
Gazebo - https://drive.google.com/file/d/1wuF1uHurAKNTdDoedqNu92J-KFQa0sA7/view?usp=drive_link
RViz - https://drive.google.com/file/d/10BDPECgCdA4g73iMA6NoSzTtXgRw7n7M/view?usp=drive_link


## Окружение

- Ubuntu 22.04, **ROS 2 Humble**, **Gazebo Classic 11**
- Код: Python 3.10 (`rclpy`), launch-файлы на Python

## Зависимости

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-xacro ros-humble-rviz2 ros-humble-tf2-tools \
  python3-colcon-common-extensions python3-yaml
```

## Запуск (одной командой)

```bash
git clone <URL репозитория> ab_ws && cd ab_ws
./run.sh
```

`run.sh` собирает пакет и запускает всё: Gazebo, SLAM, Nav2, RViz и миссию A → B. Миссия стартует автоматически, когда готовы GPS, SLAM и Nav2 (около 20–30 с). В терминале должны появиться `Еду к WP1`, `WP1 достигнута`, … и `МИССИЯ ВЫПОЛНЕНА. Ошибка до B: ...`.

Параметры: `./run.sh gui:=false` (без окна Gazebo, легче для слабых машин), `rviz:=false`, `mission:=false` (без автомиссии).

## Структура

```
src/ab_nav/
├── ab_nav/gps_waypoint_nav.py   # нода миссии: GPS-точки -> цели Nav2
├── launch/sim.launch.py         # запуск всего стека
├── urdf/ab_bot.urdf.xacro       # робот и сенсоры
├── worlds/ab.world              # мир: арена, препятствия, зоны A и B
└── config/                      # nav2_params, slam_params, waypoints
```

## Робот и сенсоры

Собственный URDF/Xacro: дифференциальный привод, два ведущих колеса и две опоры.

| Сенсор | Плагин Gazebo | Параметры | Топик |
|---|---|---|---|
| 2D-лидар | `libgazebo_ros_ray_sensor.so` | 360 лучей, 0.12–6 м, 10 Гц | `/scan` |
| Depth-камера (аналог Astra) | `libgazebo_ros_camera.so` | 60° по горизонтали, 0.6–5 м, 160×120, 10 Гц | `/astra/image_raw`, `/astra/depth/image_raw`, `/astra/points` |
| GPS | `libgazebo_ros_gps_sensor.so` | 5 Гц | `/gps/fix` |
| Привод и одометрия | `libgazebo_ros_diff_drive.so` | | `/cmd_vel`, `/odom` |

Разрешение depth занижено для слабых машин.

## TF и узлы

TF: `map → odom` (slam_toolbox) → `base_footprint` (diff_drive) → `base_link` → `lidar_link`, `camera_link` (+ `camera_depth_optical_frame`), `gps_link`.

Узлы: `gzserver`/`gzclient`, `robot_state_publisher`, `slam_toolbox`, Nav2 (`planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `smoother_server`, `velocity_smoother`, `lifecycle_manager`), `gps_waypoint_nav`, `rviz2`.

## Как это работает

1. **GPS-waypoints.** В мире задан `spherical_coordinates` (55.75, 37.6 = точка A). Файл `config/waypoints.yaml` содержит точки в широте и долготе. Нода `gps_waypoint_nav` усредняет первые GPS-фиксы (это начало координат карты), переводит lat/lon каждой точки в метры (восток, север) и отправляет их в Nav2 как цели `NavigateToPose` в системе `map`. Допущение: карта SLAM начинается в точке старта, а ось x робота при старте смотрит на восток. После каждой цели нода проверяет по TF, что робот реально у цели (допуск 0.5 м).
2. **Карта и положение.** `slam_toolbox` строит карту по лидару и даёт `map → odom`. Положение робота во время езды определяют SLAM и одометрия, GPS используется для задания маршрута и стартовой привязки.
3. **Глобальный путь.** `SmacPlanner2D` (A* на сетке) на глобальной карте препятствий со скользящим окном 30×30 м вокруг робота, данные лидара, зона инфляции вокруг препятствий. Границы карты SLAM для планирования не используются, поэтому цель может лежать за пределами уже исследованной области.
4. **Локальное движение и объезд.** `DWB` выбирает скорости по траекториям на 1.2 с вперёд. Локальная карта препятствий строится по лидару и облаку точек depth-камеры (`voxel_layer`).

## Мир

Арена 16×8 м со стенами, 5 статических препятствий (две коробки, два цилиндра, одна стенка), зелёная площадка A (старт) и красная B (финиш, 12 м от A).

## Проверка сенсоров

```bash
ros2 topic hz /scan
ros2 topic echo /gps/fix --once
ros2 topic list | grep astra
ros2 run tf2_tools view_frames
```

## Ограничения

- Проверено на Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.
- Цели должны лежать в свободном пространстве. Между препятствиями нужен зазор не меньше ~1.2 м (радиус робота 0.18 м плюс инфляция 0.35 м).
- GPS в симуляции без шума, слияние GPS с одометрией (`robot_localization`) не реализовано.

## Бонусные вопросы (электроника)

**1. Подключение к Raspberry Pi 5.**
- Orbbec Astra: USB 3.0, драйвер OpenNI2 или `astra_camera`, лучше через USB-хаб с внешним питанием.
- Ультразвук (HC-SR04): GPIO (TRIG/ECHO), на ECHO нужен делитель или level shifter (5 В на 3.3 В вход Pi), либо датчики с UART/I²C. На Pi 5 используются `gpiod`/`lgpio`/`gpiozero`, а `RPi.GPIO` не работает.
- GPS (u-blox NEO-M8N): UART (`/dev/ttyAMA0`) или USB-UART, 3.3 В, чтение через `gpsd`, опционально линия PPS.

**2. Питание от одного аккумулятора.** LiPo 3S/4S. Понижающие DC-DC: 5 В/5 А для Pi 5, 5 В для периферии, моторы через драйвер напрямую с батареи. Общая земля, предохранитель, выключатель, защита от переполюсовки, мониторинг напряжения (INA219/ADC) с отсечкой при низком заряде, конденсаторы против просадок. Pi и моторы не питать от одного регулятора.

**3. Проверка датчиков без ROS.**
- Astra: `lsusb`, пример `SimpleViewer` из OpenNI2.
- GPS: `cat /dev/ttyAMA0` (NMEA-строки), `cgps -s` или `gpsmon` на открытом небе.
- Ультразвук: скрипт на `gpiozero.DistanceSensor` и сверка с рулеткой, при отладке осциллограф и мультиметр.
- I²C-устройства: `i2cdetect -y 1`.
EOF
