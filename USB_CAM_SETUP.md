# Адаптация VINS-Mono для USB камеры и MAVROS IMU

## Обзор конфигурации

Созданы два новых файла для вашей системы:
- **`config/usb_cam_config.yaml`** - конфигурация параметров
- **`vins_estimator/launch/usb_cam.launch`** - файл запуска

## Параметры камеры

### Разрешение и калибровка
```
Разрешение: 640x480
Матрица камеры K:
  fx = 473.41 пикселей
  fy = 473.57 пикселей
  cx = 311.40 пикселей (центр по X)
  cy = 256.94 пикселей (центр по Y)

Дистortion (модель plumb_bob):
  k1 = 0.0516  (радиальное искажение)
  k2 = -0.0484 (радиальное искажение)
  p1 = 0.0047  (тангенциальное искажение)
  p2 = -0.0033 (тангенциальное искажение)
```

## Экстринсики камеры-IMU (Extrinsics)

### Геометрия
Ваша система имеет следующее расположение:
```
IMU координаты:    Камера повернута на 90° вправо
X - forward        относительно IMU
Y - left
Z - up
```

### Матрица вращения (Rotation Matrix)
Поворот камеры на -90° вокруг оси Z (вправо по курсу):
```
imu^R_cam = [0  1  0]
            [-1 0  0]
            [0  0  1]

Преобразование: imu_point = R_cam * camera_point
```

### Вектор трансляции (Translation)
```
imu^T_cam = [0, 0, 0]^T  (камера находится в центре IMU)
```

Если камера смещена относительно IMU, отредактируйте значения в строках:
```yaml
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [dx, dy, dz]  # Смещения в метрах
```

## ROS Топики

### Входящие сигналы:
- **Изображения:** `/usb_cam/image_raw`
- **Данные IMU:** `/mavros/imu/data`

### Исходящие сигналы:
- `/vins_estimator/odometry` - визуально-инерциальная одометрия
- `/vins_estimator/path` - траектория движения
- `/vins_estimator/relo_relative_pose` - релокализация
- `/feature_tracker/feature` - отслеживаемые признаки (для отладки)
- `/pose_graph/base_path` - оптимизированная траектория

## Запуск системы

### 1. Сборка проекта
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Запуск USB камеры (если еще не запущена)
```bash
roslaunch usb_cam usb_cam-test.launch
# или
rosrun usb_cam usb_cam_node
```

### 3. Запуск MAVROS
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
# или другой адрес вашего автопилота
```

### 4. Запуск VINS-Mono с USB камерой
```bash
roslaunch vins_estimator usb_cam.launch
```

В другом терминале для визуализации:
```bash
roslaunch vins_estimator vins_rviz.launch
```

## Параметры, которые можно настроить

### Отслеживание признаков
Файл: `config/usb_cam_config.yaml`
```yaml
max_cnt: 150          # Максимум отслеживаемых точек (↑ = точнее, но медленнее)
min_dist: 30          # Минимальное расстояние между точками (в пиксе)
freq: 10              # Частота обновления (Hz)
F_threshold: 1.0      # Порог RANSAC (пиксели)
show_track: 1         # Показывать отслеживаемые точки
equalize: 1           # Выравнивание гистограммы (для темных/светлых изображений)
```

### Оптимизация
```yaml
max_solver_time: 0.04     # Макс время решения (мс) для реалтайма
max_num_iterations: 8     # Макс итераций оптимизатора
keyframe_parallax: 10.0   # Порог параллакса для выбора ключевого кадра
```

### IMU параметры
```yaml
acc_n: 0.08          # Шум акселерометра
gyr_n: 0.004         # Шум гироскопа
acc_w: 0.00004       # Дрейф акселерометра
gyr_w: 2.0e-6        # Дрейф гироскопа
g_norm: 9.81007      # Ускорение свободного падения
```

Эти значения подобраны для типичного 9-DOF IMU. Если результаты неудовлетворительны, их можно調整.

### Замыкание цикла
```yaml
loop_closure: 1                  # Включить обнаружение замыкания цикла
fast_relocalization: 0           # Быстрая релокализация (для больших карт)
load_previous_pose_graph: 0      # Переиспользовать старый граф
```

## Отладка и тестирование

### Проверить топики
```bash
rostopic list           # Список всех топиков
rostopic echo /usb_cam/image_raw
rostopic echo /mavros/imu/data
rostopic echo /vins_estimator/odometry
```

### Проверить частоту обновления
```bash
rostopic hz /usb_cam/image_raw
rostopic hz /mavros/imu/data
rostopic hz /vins_estimator/odometry
```

### Визуализация в RVIZ
```bash
# Откроется RVIZ с предконфигурированными трансформациями
roslaunch vins_estimator vins_rviz.launch
```

Добавьте подписки на топики:
- `Image` → `/usb_cam/image_raw`
- `Odometry` → `/vins_estimator/odometry`
- `Path` → `/vins_estimator/path`
- `Point Cloud` → `/feature_tracker/feature` (для отладки)

## Синхронизация камеры-IMU

Если камера и IMU работают с разными временными метками:

1. **Включите автоматическую временную калибровку:**
```yaml
estimate_td: 1        # вместо 0
td: 0.0               # начальное смещение времени
```

2. **Система будет автоматически калибровать временное смещение** между камерой и IMU

## Проблемы и решения

### Проблема: "Waiting for features..."
**Решение:** Убедитесь, что:
- Камера получает достаточно света
- `max_cnt` не слишком мал
- `equalize: 1` для темных изображений

### Проблема: "Cannot find corresponding features"
**Решение:**
- Уменьшите `min_dist` (минимальное расстояние между точками)
- Увеличьте `max_cnt` (максимум точек)
- Проверьте калибровку камеры

### Проблема: "IMU data drift"
**Решение:**
- Отредактируйте параметры `acc_w` и `gyr_w` в конфиге
- Используйте более точный IMU

### Проблема: Неправильная ориентация камеры
**Решение:** Отредактируйте матрицу вращения в конфиге:
```yaml
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [...]  # Измените эту матрицу
```

## Сохранение и загрузка карты

### Сохранить текущую карту:
В терминале `vins_estimator` нажмите **`s` + Enter**

### Загрузить сохраненную карту:
Установите в конфиге:
```yaml
load_previous_pose_graph: 1
pose_graph_save_path: "/path/to/saved/pose_graph/"
```

## Контрольный список перед запуском

- [ ] Откалибрована USB камера (параметры K, D обновлены)
- [ ] MAVROS работает и публикует `/mavros/imu/data`
- [ ] USB камера работает и публикует `/usb_cam/image_raw`
- [ ] Проверены топики: `rostopic list | grep -E "usb_cam|mavros|imu"`
- [ ] Созданы директории вывода: `mkdir -p /home/op/output/pose_graph/`
- [ ] Выполнена сборка: `cd ~/catkin_ws && catkin_make`

## Справка по файлам

- **Основной конфиг:** `config/usb_cam_config.yaml`
- **Launch файл:** `vins_estimator/launch/usb_cam.launch`
- **Код оценивателя:** `vins_estimator/src/estimator_node.cpp`
- **Отслеживание признаков:** `feature_tracker/src/feature_tracker_node.cpp`
- **Граф позиций:** `pose_graph/src/pose_graph_node.cpp`

## Дополнительные ресурсы

- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Mono Paper (IEEE)](https://ieeexplore.ieee.org/document/8421746)
- [ROS Navigation](http://wiki.ros.org/ROS/Tutorials)
