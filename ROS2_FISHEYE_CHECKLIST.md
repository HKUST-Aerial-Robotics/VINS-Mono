# ROS2 fisheye bag checklist

Этот чек-лист подходит для запуска `vins_bringup` с bag, где входная картинка идет в формате `sensor_msgs/msg/CompressedImage`.

## 1. Перед запуском

Убедись, что bag уже запущен и публикует:

- `/image_raw/compressed`
- `/mavros/imu/data`

Проверка:

```bash
source ~/.bashrc
ros2 topic list | grep -E 'image_raw/compressed|mavros/imu/data'
ros2 topic hz /image_raw/compressed
ros2 topic hz /mavros/imu/data
```

Ожидаемо:

- `/image_raw/compressed` есть
- `/mavros/imu/data` есть
- частота у обоих топиков не нулевая

## 2. Конфиг

Для bag-режима в `config/fisheye_bag/fisheye_bag_config.yaml` должны быть:

```yaml
image_topic: "/image_raw/compressed"
image_input_format: "compressed"
imu_topic: "/mavros/imu/data"
estimate_extrinsic: 0
extrinsicTranslation: [0.0, 0.0, 0.0]
```

Если позже захочешь вернуть обычную raw-камеру:

```yaml
image_topic: "/camera/fisheye/image_raw"
image_input_format: "raw"
```

## 3. Запуск

В одном терминале:

```bash
source ~/.bashrc
ros2 launch vins_bringup innospector_cam.launch.py
```

## 4. Что должно появиться в консоли

### feature_tracker

Должны быть строки вроде:

```text
[START] image_input_format=compressed
[START] compressed subscription created for /image_raw/compressed
[INPUT] first compressed image received: ...
[INPUT] first image received: ...
[STATE] tracker primed on first frame
[OUTPUT] published feature packet #1 ...
```

### vins_estimator

Должны быть строки вроде:

```text
[START] vins_estimator is up and waiting for synchronized measurement pairs
[INPUT] imu #1 ...
[INPUT] first feature packet received: ...
```

### pose_graph

Нормально увидеть:

```text
no previous pose graph
```

## 5. Проверка топиков после старта

В другом терминале:

```bash
source ~/.bashrc
ros2 topic list | grep -E 'image_raw/compressed|feature_tracker|vins_estimator|pose_graph'
ros2 topic info -v /image_raw/compressed
ros2 topic hz /image_raw/compressed
ros2 topic hz /feature_tracker/feature
ros2 topic hz /feature_tracker/feature_img
ros2 topic hz /vins_estimator/odometry
```

Ожидаемо:

- `/image_raw/compressed` публикуется rosbag с типом `sensor_msgs/msg/CompressedImage`
- `feature_tracker` подписан на него через transport `compressed`
- `/feature_tracker/feature` имеет ненулевую частоту
- `/feature_tracker/feature_img` имеет частоту около заданного `freq` и тип `sensor_msgs/msg/Image`
- `/vins_estimator/odometry` появляется после того, как наберется достаточно кадров

## 6. Если что-то не работает

### Сценарий A: есть IMU, но нет image callback

Это значит, что `feature_tracker` не получает картинку.

Проверь:

```bash
ros2 topic echo --once /image_raw/compressed
ros2 topic info -v /image_raw/compressed
ros2 node list
```

### Сценарий B: `feature_tracker` получает картинку, но `vins_estimator` пишет `queue_feat=0`

Это значит, что `feature_tracker` не публикует `feature`.

Проверь логи `feature_tracker`:

```text
[OUTPUT] published feature packet ...
```

### Сценарий C: `vins_estimator` пишет `imu message in disorder!`

Это обычно проблема порядка временных меток.

Проверь:

```bash
ros2 topic hz /mavros/imu/data
ros2 topic echo --once /mavros/imu/data
```

### Сценарий D: launch стартует, но видны только старые предупреждения FastDDS

Сообщения вида:

```text
RTPS_TRANSPORT_SHM Error
```

часто не критичны для логики VINS, но могут мешать диагностике.  
Главное — есть ли реальные сообщения на нужных топиках.

## 7. Быстрая команда “все ли живо”

```bash
source ~/.bashrc
ros2 topic hz /image_raw/compressed
ros2 topic hz /mavros/imu/data
ros2 topic hz /feature_tracker/feature
ros2 topic hz /vins_estimator/odometry
```

Если первые две частоты есть, а последние две нет — проблема между камерой и VINS.

## 8. Диагностические сообщения новой цепочки

При нормальной работе последовательно появляются:

```text
[INPUT] first image received ... encoding=mono8
[OUTPUT] published feature packet ...
[OUTPUT] published feature image ... /feature_tracker/feature_img
[INPUT] first feature packet received ...
[OUTPUT] accepted keyframe ...
```

Сообщения `[FLOW]`, `inconsistent feature vectors`, `non-increasing image timestamps`,
`invalid keyframe point layout` и `[SYNC] ... exhausted` означают конкретное место разрыва.

Unit-тесты защитных проверок:

```bash
cd ~/ros2_ws
source ~/.bashrc
colcon test --packages-select feature_tracker --event-handlers console_direct+
colcon test-result --verbose
```
