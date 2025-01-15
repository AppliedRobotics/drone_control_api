# Drone Control API

## Описание

Drone Control API — это Python-пакет для дистанционного управления образовательным дроном **DroneCam EDU**. Пакет предоставляет простой и интуитивно понятный интерфейс для взаимодействия с дроном через WebSockets, позволяя выполнять основные действия, такие как взлет, посадка, управление скоростью и направление, а также получать данные от датчиков и камеры дрона.

## Содержание

- [Особенности](#особенности)
- [Установка](#установка)
- [Зависимости](#зависимости)
- [Быстрый старт](#быстрый-старт)
- [Примеры использования](#примеры-использования)
- [API Reference](#api-reference)
- [Вклад](#вклад)
- [Лицензия](#лицензия)

## Особенности

- **Подключение через WebSockets:** Надежное и быстрое соединение с дроном.
- **Управление полетом:** Взлет, посадка, управление скоростью и направлением.
- **Получение данных:** Доступ к данным GPS, барометру, ультразвуковым датчикам и другим сенсорам.
- **Работа с камерой:** Получение изображений с камеры дрона для анализа или отображения.
- **Асинхронность:** Поддержка асинхронных операций для эффективного управления ресурсами.

## Установка

Установка пакета осуществляется через `pip`. Убедитесь, что у вас установлен Python версии 3.6 или выше.

```bash
pip3 install drone_control_api --break-system-packages
```


## Зависимости

Пакет требует наличия следующих зависимостей:

- `websockets`
- `opencv-python`
- `numpy`

Эти зависимости автоматически устанавливаются вместе с пакетом. Однако, вы также можете установить их вручную, используя файл `requirements.txt`:

```bash
pip3 install -r requirements.txt
```


**Примечание:** Файл `requirements.txt` в данном проекте пуст. Вам необходимо добавить необходимые зависимости или использовать вышеуказанную команду для их установки.

## Быстрый старт

### Подключение к дрону

Ниже приведен пример того, как подключиться к дрону и выполнить основные действия:

```python
from drone_control_api import Drone

client = Drone()
```  

### Запуск примеров

В директории `examples/` находится скрипт `example.py`, демонстрирующий использование пакета. Запустите его следующей командой:

```bash
python3 examples/example.py
```



## API Reference

### Класс `Drone`

Основной класс для управления дроном.

#### Методы

- **connect(ip: str, port: str) -> bool**

  Подключается к дрону по указанному IP и порту.

- **disconnect() -> bool**

  Отключается от дрона.

- **takeoff()**

  Выполняет взлет.

- **boarding()**

  Выполняет посадку.

- **setYaw(yaw: float)**
  
  Устанавливает угол поворота дрона.

- **setVelXY(x: float, y: float)**
  
  Устанавливает скорость дрона по осям X и Y.

- **gotoXYdrone(x: float, y: float)**
  
  Перемещает дрон в заданные координаты.

- **gotoXYodom(x: float, y: float)**
  
  Перемещает дрон относительно текущей позиции.

- **setHeight(height: float)**
  
  Устанавливает высоту дрона.

- **setMagnet(val: bool)**
  
  Включает или отключает магнит.

- **setDiod(r: float, g: float, b: float)**
  
  Устанавливает цвет светодиодов.

- **setBeeper(power: float, freq: float)**
  
  Включает зумер с заданной мощностью и частотой.

- **getImage() -> numpy.ndarray**
  
  Получает изображение с камеры дрона.

- **getUtilsData() -> dict**
  
  Получает данные утилит.

- **setZeroOdomOpticflow()**
  
  Устанавливает нулевую точку отсчета для оптического потока.

- **getOdomOpticflow() -> dict**
  
  Получает данные одометрии от оптического потока.

- **getLidar() -> dict**
  
  Получает данные с лидара.

- **getRPY() -> dict**
  
  Получает углы крена (Roll), тангажа (Pitch) и рыскания (Yaw).

- **getHeightBarometer() -> float**
  
  Получает высоту по барометру.

- **getHeightRange() -> float**
  
  Получает высоту по дальномеру.

- **getArm() -> bool**
  
  Получает состояние моторов дрона (вкл/выкл).

- **getArucos() -> dict**
  
  Получает информацию об обнаруженных ArUco-маркерах.

- **getCameraPoseAruco() -> dict**
  
  Получает позицию камеры относительно ArUco-маркеров.

- **getLight() -> dict**
  
  Получает данные с датчиков освещенности.

- **getUltrasonic() -> dict**
  
  Получает данные с ультразвуковых датчиков.

- **getBlobs() -> dict**
  
  Получает информацию об обнаруженных цветовых пятнах.

- **setVelXYYaw(x: float, y: float, yaw: float)**
  
  Устанавливает скорость движения по осям X, Y и скорость поворота вокруг оси Z.

#### Обработчики сообщений

- **AddOnMessangeUtils(handler: Callable)**

  Добавляет обработчик для сообщений утилит.

- **AddOnMessangeImage(handler: Callable)**

  Добавляет обработчик для сообщений изображений.

## Вклад

Мы приветствуем вклад сообщества в развитие проекта! Если вы нашли ошибку или хотите предложить новую функциональность, пожалуйста, создайте [issue](https://github.com/applied_robotics/drone_control_api/issues) или сделайте [pull request](https://github.com/applied_robotics/drone_control_api/pulls).

## Лицензия

Этот проект лицензирован под лицензией MIT. Подробности см. в файле [LICENSE](https://github.com/applied_robotics/drone_control_api/blob/main/LICENSE).
