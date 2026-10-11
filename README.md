# Лабораторна робота №2 — Gazebo + ArduPilot SITL + ROS 2 (варіант 9)

Політ дрона до точки (x0 − 5,2; y0 + 0,1) з подальшою автоматичною посадкою
(Δx = 5,2 м, Δy = −0,1 м).

## Середовище
Debian 13 (хост), distrobox `ros2_humble` (Ubuntu 22.04), ROS 2 Humble,
Gazebo Harmonic (Sim 8.15.0), ArduPilot SITL `ArduPilot-4.6.0-beta1-9182-gd67add0b9b`.

## Структура
- `src/fpv_lab2/flight_test/` — наданий приклад (`flight_test_node.py`, `handlers/`, `main.py`)
  та власний код: `navigator.py` (алгоритм наведення), `target_node.py` (вузол польоту до цілі),
  `target_main.py`, стани NAVIGATING/STOPPING у `flight_state.py`
- `src/fpv_lab2/test/test_navigator.py` — перевірка логіки навігатора без Gazebo
- `scripts/` — скрипти запуску і перевірки середовища
- `docs/` — журнали та знімки екрана

## Запуск
```bash
distrobox enter ros2_humble
cd ~/fpv_labs
./scripts/build.sh              # збірка пакета fpv_lab2
./scripts/start_env.sh          # Gazebo + ArduPilot SITL + DDS
./scripts/check_env.sh          # перевірка процесів, сервісів, тем
./scripts/run_lab.sh example    # початковий приклад: зліт-зависання-посадка
./scripts/run_lab.sh target     # політ до цілі (варіант 9)
./scripts/stop_env.sh           # зупинка
```

Похибка: `python3 scripts/calc_error.py X_ФАКТ Y_ФАКТ`.

## Особливості
- Після зупинки SITL порт 5760 деякий час зайнятий — `start_env.sh` викликає `wait_ports.sh`.
- `check_env.sh` одразу після запуску може показати FAIL: DDS реєструє сервіси із затримкою.
- Програмний рендеринг (llvmpipe): Gazebo працює повільно.
