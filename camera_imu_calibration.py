#!/usr/bin/env python3
"""
Интерактивный инструмент для определения матрицы вращения камеры-IMU.

Использование:
1. Запустите этот скрипт
2. Следуйте инструкциям и отвечайте на вопросы о расположении осей
3. Получите матрицу вращения для конфига
"""

import numpy as np
from scipy.spatial.transform import Rotation
import sys

def print_section(title):
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def get_axis_direction(sensor_name, axis_name):
    """Получить направление оси от пользователя"""
    directions = {
        'x': 'X',
        'y': 'Y', 
        'z': 'Z',
        '+x': '+X (forward)',
        '-x': '-X (backward)',
        '+y': '+Y (right)',
        '-y': '-Y (left)',
        '+z': '+Z (up)',
        '-z': '-Z (down)'
    }
    
    print(f"\n{sensor_name} - Ось {axis_name}:")
    print("  Введите направление этой оси:")
    print("  +x = forward,  -x = backward")
    print("  +y = right,    -y = left")
    print("  +z = up,       -z = down")
    print("  (пример: +z, -y, +x)")
    
    while True:
        response = input(f"  {sensor_name} {axis_name} направлена в: ").strip().lower()
        if response in directions:
            return response
        print("  ❌ Некорректный ввод. Попробуйте снова.")

def direction_to_vector(direction):
    """Преобразовать направление в вектор"""
    mapping = {
        '+x': np.array([1, 0, 0]),
        '-x': np.array([-1, 0, 0]),
        '+y': np.array([0, 1, 0]),
        '-y': np.array([0, -1, 0]),
        '+z': np.array([0, 0, 1]),
        '-z': np.array([0, 0, -1]),
    }
    return mapping[direction]

def get_imu_axes():
    """Получить оси IMU"""
    print_section("КОНФИГУРАЦИЯ IMU")
    print("\nОпределите направление осей IMU в пространстве.")
    print("(Как правило X-forward, Y-left/right, Z-up)")
    
    imu_x = get_axis_direction("IMU", "X")
    imu_y = get_axis_direction("IMU", "Y")
    imu_z = get_axis_direction("IMU", "Z")
    
    return imu_x, imu_y, imu_z

def get_camera_axes():
    """Получить оси камеры"""
    print_section("КОНФИГУРАЦИЯ КАМЕРЫ")
    print("\nОпределите направление осей камеры в пространстве.")
    print("(Стандартная ориентация камеры: Z-forward, X-right, Y-down)")
    
    cam_x = get_axis_direction("КАМЕРА", "X")
    cam_y = get_axis_direction("КАМЕРА", "Y")
    cam_z = get_axis_direction("КАМЕРА", "Z")
    
    return cam_x, cam_y, cam_z

def verify_orthogonal(v1, v2, v3, name):
    """Проверить ортогональность векторов"""
    # Проверить что все векторы разные
    if np.allclose(np.abs(v1), np.abs(v2)) or np.allclose(np.abs(v1), np.abs(v3)) or np.allclose(np.abs(v2), np.abs(v3)):
        print(f"\n❌ ОШИБКА в {name}: Две оси указывают на одно направление!")
        return False
    
    # Проверить ортогональность (скалярное произведение должно быть 0)
    dot12 = np.dot(v1, v2)
    dot13 = np.dot(v1, v3)
    dot23 = np.dot(v2, v3)
    
    tol = 1e-6
    if abs(dot12) > tol or abs(dot13) > tol or abs(dot23) > tol:
        print(f"\n❌ ОШИБКА в {name}: Оси не ортогональны!")
        return False
    
    return True

def vectors_to_rotation_matrix(cam_x_dir, cam_y_dir, cam_z_dir, imu_x_dir, imu_y_dir, imu_z_dir):
    """
    Вычислить матрицу вращения от камеры к IMU.
    
    Входные данные:
    - cam_x_dir, cam_y_dir, cam_z_dir: направления осей камеры в мировых координатах
    - imu_x_dir, imu_y_dir, imu_z_dir: направления осей IMU в мировых координатах
    
    Выход:
    - R: матрица вращения такая что imu_point = R * camera_point
    """
    
    # Матрица, где столбцы - это базис камеры в мировых координатах
    camera_basis = np.column_stack([cam_x_dir, cam_y_dir, cam_z_dir])
    
    # Матрица, где столбцы - это базис IMU в мировых координатах
    imu_basis = np.column_stack([imu_x_dir, imu_y_dir, imu_z_dir])
    
    # R преобразует вектор из камеры в IMU
    # мир = camera_basis @ cam_coords
    # мир = imu_basis @ imu_coords
    # Поэтому: imu_basis @ imu_coords = camera_basis @ cam_coords
    # imu_coords = (imu_basis^-1 @ camera_basis) @ cam_coords
    # R = imu_basis^-1 @ camera_basis
    
    R = np.linalg.inv(imu_basis) @ camera_basis
    
    return R

def print_matrix_in_yaml(R):
    """Вывести матрицу в формате YAML для конфига"""
    print_section("РЕЗУЛЬТАТ: МАТРИЦА ВРАЩЕНИЯ")
    print("\nКопируйте эту матрицу в config/usb_cam_config.yaml:")
    print("\nextrinsicRotation: !!opencv-matrix")
    print("   rows: 3")
    print("   cols: 3")
    print("   dt: d")
    print("   data: [", end="")
    
    # Вывести в одну строку для копирования
    flat = R.flatten()
    for i, val in enumerate(flat):
        # Округлить до 6 знаков после запятой
        print(f"{val:.15f}", end="")
        if i < len(flat) - 1:
            print(", ", end="")
    print("]")
    
    print("\nИли в развёрнутом виде:")
    print("extrinsicRotation: !!opencv-matrix")
    print("   rows: 3")
    print("   cols: 3")
    print("   dt: d")
    print("   data: [" + f"{R[0,0]:.6f}, {R[0,1]:.6f}, {R[0,2]:.6f},")
    print(f"          {R[1,0]:.6f}, {R[1,1]:.6f}, {R[1,2]:.6f},")
    print(f"          {R[2,0]:.6f}, {R[2,1]:.6f}, {R[2,2]:.6f}]")

def print_verification(cam_x, cam_y, cam_z, imu_x, imu_y, imu_z, R):
    """Напечатать проверку преобразований"""
    print_section("ПРОВЕРКА ПРЕОБРАЗОВАНИЙ")
    
    print("\nОси КАМЕРЫ в МИРОВЫХ координатах:")
    print(f"  X: {cam_x} → {direction_to_vector(cam_x)}")
    print(f"  Y: {cam_y} → {direction_to_vector(cam_y)}")
    print(f"  Z: {cam_z} → {direction_to_vector(cam_z)}")
    
    print("\nОси IMU в МИРОВЫХ координатах:")
    print(f"  X: {imu_x} → {direction_to_vector(imu_x)}")
    print(f"  Y: {imu_y} → {direction_to_vector(imu_y)}")
    print(f"  Z: {imu_z} → {direction_to_vector(imu_z)}")
    
    print("\nМатрица вращения imu^R_cam:")
    print(R)
    
    print("\nПроверка: преобразование осей камеры через R:")
    cam_x_vec = direction_to_vector(cam_x)
    cam_y_vec = direction_to_vector(cam_y)
    cam_z_vec = direction_to_vector(cam_z)
    
    result_x = R @ cam_x_vec
    result_y = R @ cam_y_vec
    result_z = R @ cam_z_vec
    
    imu_x_vec = direction_to_vector(imu_x)
    imu_y_vec = direction_to_vector(imu_y)
    imu_z_vec = direction_to_vector(imu_z)
    
    print(f"\n  R @ cam_X = {result_x} (должно быть {imu_x_vec} - IMU X)")
    print(f"  R @ cam_Y = {result_y} (должно быть {imu_y_vec} - IMU Y)")
    print(f"  R @ cam_Z = {result_z} (должно быть {imu_z_vec} - IMU Z)")
    
    # Проверить корректность
    if (np.allclose(result_x, imu_x_vec) and 
        np.allclose(result_y, imu_y_vec) and 
        np.allclose(result_z, imu_z_vec)):
        print("\n✅ Матрица ВЕРНА! Оси правильно преобразуются.")
    else:
        print("\n❌ Что-то не так. Проверьте введённые данные.")

def main():
    print_section("КАЛИБРОВКА МАТРИЦЫ ВРАЩЕНИЯ КАМЕРЫ-IMU")
    print("\nЭтот инструмент поможет вам определить правильную матрицу вращения")
    print("между камерой и IMU на основе фактического расположения осей.")
    
    # Получить оси IMU
    imu_x, imu_y, imu_z = get_imu_axes()
    
    # Получить оси камеры
    cam_x, cam_y, cam_z = get_camera_axes()
    
    # Преобразовать в векторы
    cam_x_vec = direction_to_vector(cam_x)
    cam_y_vec = direction_to_vector(cam_y)
    cam_z_vec = direction_to_vector(cam_z)
    
    imu_x_vec = direction_to_vector(imu_x)
    imu_y_vec = direction_to_vector(imu_y)
    imu_z_vec = direction_to_vector(imu_z)
    
    # Проверить ортогональность
    if not verify_orthogonal(cam_x_vec, cam_y_vec, cam_z_vec, "КАМЕРА"):
        sys.exit(1)
    if not verify_orthogonal(imu_x_vec, imu_y_vec, imu_z_vec, "IMU"):
        sys.exit(1)
    
    # Вычислить матрицу вращения
    R = vectors_to_rotation_matrix(cam_x_vec, cam_y_vec, cam_z_vec, 
                                    imu_x_vec, imu_y_vec, imu_z_vec)
    
    # Вывести результаты
    print_verification(cam_x, cam_y, cam_z, imu_x, imu_y, imu_z, R)
    print_matrix_in_yaml(R)
    
    print_section("ДАЛЕЕ")
    print("\n1. Скопируйте матрицу выше в файл config/usb_cam_config.yaml")
    print("2. Пересоберите: cd ~/catkin_ws && catkin_make")
    print("3. Перезапустите VINS: roslaunch vins_estimator usb_cam.launch")

if __name__ == "__main__":
    main()
