[СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[МЭК-61107/IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Энергомера МЭК/IEC](https://github.com/latonita/esphome-energomera-iec) •
[Энергомера CE](https://github.com/latonita/esphome-energomera-ce) •
[СПб ЗИП ЦЭ2727А](https://github.com/latonita/esphome-ce2727a-meter) •
[Ленэлектро ЛЕ-2](https://github.com/latonita/esphome-le2-meter) •
[Пульсар-М](https://github.com/latonita/esphome-pulsar-m) •
[Энергомера BLE](https://github.com/latonita/esphome-energomera-ble)

# ESPHome компонент для подключения однофазных счетчиков электроэнергии ЛЕНЭЛЕКТРО ЛЕ-2 через оптопорт или RS-485

* [1. Назначение](#1-назначение)
* [2. Отказ от ответственности](#2-отказ-от-ответственности)
* [3. Функции](#3-функции)
* [4. Пример отображения в home-assistant](#4-пример-отображения-в-home-assistant)
* [5. Подключение](#5-подключение)
* [6. Настройка основного компонента](#6-настройка-основного-компонента)
* [7. Настройка сенсоров для опроса счетчика](#7-настройка-сенсоров-для-опроса-счетчика)
* [Приложение. Коды ошибок ПУ](#приложение-коды-ошибок-пу)

## 1. Назначение
Компонент для считывания данных с однофазных электросчетчиков (приборов учета, ПУ) производства ЛЕНЭЛЕКТРО ЛЕ-2 (D5 и D6) по протоколу Ленэлектро.

Поддерживаемые модели:

| Тип | Модель |
|---|--------|
| 0 | ЛЕ-2 4.1/2.ОR4RF3.D6.A2R2.SLRMUs.Le5(60) |
| 1 | ЛЕ-2 4.1/2.ОR4NB1.D6.A2R2.SLRMUs.Sp5(60) |
| 2 | ЛЕ-2 4.1/2.ОR4.D6.A2R2.SLRMUs.Le5(60) |
| 3 | ЛЕ-2 4.1/2.ORF3.D5.A2R2.SLRMUs.Le5(60) |

* Для подключения к типу 1 возможно использовать компонент СПОДЭС https://github.com/latonita/esphome-dlms-cosem/ 

## 2. Отказ от ответственности
Пользуясь данным ПО пользователь полностью берет на себя всю ответственность за любые последствия.
 
## 3. Функции
- подключение как безадресное (широковещательный запрос), так и по адресу (обычно это 9 последних цифр заводского номера, либо весь номер),
- считывание параметров сети и текущих накоплений

## 4. Пример отображения в home-assistant
![Пример отображения в home-asistant](/images/le2.png) 

## 5. Подключение
Инструкции по подключению esp32/esp8266 к счётчику можно увидеть в соседнем компоненте https://github.com/latonita/esphome-energomera-iec

## 6. Настройка основного компонента
Подлючаем внешний компонент из репозитория
```yaml
external_components:
  - source: github://latonita/esphome-le2-meter
    refresh: 30s
    components: [le2]
```
Для оптоголовки конфигурируем UART 9600 8N1:
```yaml
uart:
  rx_pin: GPIO16
  tx_pin: GPIO17
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1
```

Основной модуль (hub)
```yaml
le2:
  password: 11111111   # пароль по-умолчанию, см. инструкцию/паспорт
  receive_timeout: 500ms
  #address: 0
  #flow_control_pin: 27 
```
- `address` - по-умолчанию пустой, если счетчик один - то адрес не требуется. Если несколько счетчиков - то там указываем его адрес - это последние 9 цифр его заводского номера либо номер целиком.
- `receive_timeout` - по-умолчанию 500мс, если ответы длинные - то можем не успеть дождаться ответа - увеличиваем.
- `flow_control_pin` - указываем, если 485 модуль требует сигнал направления передачи RE/DE 
- `uart_id` - если использьзуете несколько портов UART, указать его id

## 7. Настройка сенсоров для опроса счетчика
Крайне не рекомедуется использовать в конфигах esphome ничего, кроме латиницы. Если необходимы названия сенсоров на русском языке - переименуйте их уже внутри home assistant.

Максимально полный набор сенсоров для двухтарифного учета:

```yaml

sensor:
  - platform: le2
    frequency: "Frequency"
    voltage: "Voltage"
    tariff_1:
      import_active_energy: "T1 Import Active Energy"
      export_active_energy: "T1 Export Active Energy"
      import_reactive_energy: "T1 Import Reactive Energy"
      export_reactive_energy: "T1 Export Reactive Energy"
    tariff_2:
      import_active_energy: "T2 Import Active Energy"
      export_active_energy: "T2 Export Active Energy"
      import_reactive_energy: "T2 Import Reactive Energy"
      export_reactive_energy: "T2 Export Reactive Energy"
    # всего 8 тарифов
    # ... 
    # tariff_8:
    # ...
    phase:
      current: "Phase Current"
      active_power: "Phase Active Power"
      reactive_power: "Phase Reactive Power"
      power_factor: "Phase Power Factor"
      apparent_power: "Phase Apparent Power"
    neutral:
      current: "Neutral Current"
      active_power: "Neutral Active Power"
      reactive_power: "Neutral Reactive Power"
      power_factor: "Neutral Power Factor"
      apparent_power: "Neutral Apparent Power"

text_sensor:
  - platform: le2
    electricity_tariff: "Current Tariff"
    date: "Date"
    time: "Time"
    datetime: "DateTime"
    network_address: "Network Address"
    serial_nr: "Serial Number"
    reading_state: "Reading State"
    error_code: "Error code"
    about: "About"
```
Сенсоры указаны в короткой нотации. При необходимости, можно делать тонкую настройку каждого сенсора как обычно, используя обычную длинную нотацию:
```yaml
sensor:
  - platform: le2
    frequency: 
      name: "Frequency"
      accuracy_decimals: 1
      filters:
        -...
    voltage: 
      name: "Voltage"
      accuracy_decimals: 0
      icon: "mdi:wave"
      # и так далее для любого сенсора 
```

## Приложение. Коды ошибок ПУ
Во время эксплуатации в ПУ могут возникать ошибки. Комбинированное значение находится в сенсоре `error_code`.
Каждому биту числа соответствует своя ошибка. При появлении ошибки необходимо связаться с эксплуатирующей организацией.

Коды ошибок счетчика ЛЕ-2 и их расшифровка представлены в [отдельном файле](errors.md).


