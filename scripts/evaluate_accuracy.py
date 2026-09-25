"""Офлайн-проверка точности по GNSS. Подробности — в README.md."""

import sys
import math
import bisect

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# --- параметры модели, как в reserve_odometry_node.py ---
K_ACCEL = 0.05
K_BRAKE = 0.08
MODEL_WEIGHT = 0.0
SLIP_THRESHOLD = 0.5
RESISTANCE_COEFFICIENT = 0.003

TIME_TOLERANCE = 0.1  # допустимое расхождение по времени, сек
GNSS_SOURCE = 'rover'  # 'master' или 'rover'


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    messages = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg_type = get_message(type_map[topic])
        messages.append((topic, deserialize_message(data, msg_type)))
    return messages


def run_model(messages):
    last_controller_pos = 0
    rear_velocity = None
    model_velocity = 0.0
    last_front_stamp = None
    estimates = []

    for topic, msg in messages:
        if topic == '/vehicle/driver_position_cmd':
            last_controller_pos = msg.position

        elif topic == '/vehicle/rear_bogie_velocity':
            rear_velocity = msg.velocity / 3.6

        elif topic == '/vehicle/front_bogie_velocity':
            now = stamp_to_sec(msg.header.stamp)

            pos = last_controller_pos
            traction_rate = K_ACCEL * pos if pos >= 0 else K_BRAKE * pos
            resistance_rate = -RESISTANCE_COEFFICIENT * model_velocity ** 2
            rate = traction_rate + resistance_rate

            dt = 0.0
            if last_front_stamp is not None:
                dt = now - last_front_stamp
                if dt < 0 or dt > 1.0:
                    dt = 0.0
            last_front_stamp = now

            model_velocity = max(0.0, model_velocity + rate * dt)

            front_velocity_ms = msg.velocity / 3.6
            odometry_velocity = front_velocity_ms
            slip_suspected = False
            if rear_velocity is not None:
                mismatch = abs(front_velocity_ms - rear_velocity)
                slip_suspected = mismatch > SLIP_THRESHOLD
                odometry_velocity = (front_velocity_ms + rear_velocity) / 2.0

            w = min(MODEL_WEIGHT + (0.4 if slip_suspected else 0.0), 1.0)
            estimated_velocity = w * model_velocity + (1.0 - w) * odometry_velocity

            estimates.append((now, estimated_velocity, odometry_velocity))

    return estimates


def read_gnss(messages, source):
    topic_name = f'/sensing/gnss/{source}/vel'
    gnss = []
    for topic, msg in messages:
        if topic == topic_name:
            t = stamp_to_sec(msg.header.stamp)
            v = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)
            gnss.append((t, v))
    gnss.sort()
    return gnss


def match_and_score(estimates, gnss, tolerance):
    gnss_times = [t for t, _ in gnss]
    gnss_values = [v for _, v in gnss]

    errors = []
    for t_est, v_est in estimates:
        idx = bisect.bisect_left(gnss_times, t_est)
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(gnss_times)]

        best_dt, best_v = None, None
        for i in candidates:
            dt = abs(gnss_times[i] - t_est)
            if best_dt is None or dt < best_dt:
                best_dt, best_v = dt, gnss_values[i]

        if best_dt is not None and best_dt <= tolerance:
            errors.append(v_est - best_v)

    return errors


def main():
    if len(sys.argv) != 2:
        print('Использование: python3 evaluate_accuracy.py <путь_к_папке_бэга>')
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f'Читаю бэг: {bag_path}')
    messages = read_bag(bag_path)
    print(f'Прочитано сообщений: {len(messages)}')

    estimates = run_model(messages)
    gnss = read_gnss(messages, GNSS_SOURCE)
    print(f'Наших оценок скорости: {len(estimates)}, GNSS-сэмплов ({GNSS_SOURCE}): {len(gnss)}')

    if not gnss:
        print(f'В этом бэге нет данных GNSS "{GNSS_SOURCE}" — выбери другой прогон.')
        sys.exit(1)

    est_vals = [v for _, v, _ in estimates]
    odo_vals = [o for _, _, o in estimates]
    gnss_vals = [v for _, v in gnss]
    print(f'Наша оценка (модель+одометрия): min={min(est_vals):.2f}  max={max(est_vals):.2f}  среднее={sum(est_vals)/len(est_vals):.2f}')
    print(f'Чистая одометрия (без модели):  min={min(odo_vals):.2f}  max={max(odo_vals):.2f}  среднее={sum(odo_vals)/len(odo_vals):.2f}')
    print(f'GNSS скорость:                  min={min(gnss_vals):.2f}  max={max(gnss_vals):.2f}  среднее={sum(gnss_vals)/len(gnss_vals):.2f}')

    errors_blend = match_and_score([(t, v) for t, v, _ in estimates], gnss, TIME_TOLERANCE)
    errors_odo = match_and_score([(t, o) for t, _, o in estimates], gnss, TIME_TOLERANCE)

    def report(name, errors):
        if not errors:
            print(f'{name}: не удалось сопоставить пары')
            return
        rmse = math.sqrt(sum(e**2 for e in errors) / len(errors))
        mae = sum(abs(e) for e in errors) / len(errors)
        print(f'{name}: RMSE={rmse:.3f} м/с, MAE={mae:.3f} м/с (пар: {len(errors)})')

    report('С моделью (текущий алгоритм)', errors_blend)
    report('Только одометрия (без модели)', errors_odo)


if __name__ == '__main__':
    main()