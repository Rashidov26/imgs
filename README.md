## Содержание
- [Введение](#введение)
- [1. Теоретические основы](#1-теоретические-основы)
- [2. Анализ исходного кода](#2-анализ-исходного-кода)
- [3. Структурные схемы и блок-схемы](#3-структурные-схемы-и-блок-схемы)
- [4. Расчётная часть](#4-расчётная-часть)
- [5. Предложение по улучшению](#5-предложение-по-улучшению)
- [Заключение](#заключение)
- [Список литературы](#список-литературы)

## Введение

Система дистанционного управления является основным интерфейсом между пилотом и беспилотным летательным аппаратом (БПЛА). От её надёжности и быстродействия зависит безопасность полёта. В данном проекте реализован приём сигнала PPM (Pulse Position Modulation) от RC‑приёмника с использованием аппаратного таймера STM32 в режиме Input Capture.

Цель работы – изучить архитектуру приёма команд, проанализировать алгоритм декодирования PPM, рассчитать временны́е параметры и предложить улучшения.

## 1. Теоретические основы

### 1.1. Форматы сигналов PWM и PPM

**PWM** (Pulse Width Modulation) – каждый канал передаётся по отдельному проводу. Длительность импульса – от 1000 до 2000 мкс, период – 20 мс. Недостаток: много проводов.

**PPM** (Pulse Position Modulation) – все каналы передаются последовательно по одному проводу. Кадр начинается синхроимпульсом длительностью >3000 мкс, затем идут импульсы каналов (1000–2000 мкс). PPM экономит вес и упрощает подключение.

В проекте PPM поступает на вывод PA0 (TIM2 Channel 1). Таймер STM32F103 тактируется частотой 72 МГц, с предделителем PSC = 71 получаем частоту счётчика 1 МГц (1 тик = 1 мкс). Режим Input Capture позволяет фиксировать значение счётчика в момент фронта сигнала без участия процессора, что даёт точность до 1 мкс.

### 1.2. Принцип работы обработчика PPM

На каждый нарастающий фронт сигнала PPM вызывается прерывание. В обработчике вычисляется длительность последнего импульса как разность между текущим захваченным значением CCR1 и предыдущим. Если происходит переполнение 16‑битного счётчика (0xFFFF → 0), разность корректируется добавлением 0xFFFF. Далее, если импульс >3000 мкс – это синхроимпульс, сбрасывается счётчик каналов. Если импульс в диапазоне 800–2200 мкс – он сохраняется в соответствующую переменную `channel_1` … `channel_10`.

### 1.3. Таймеры STM32

Таймер TIM2 настроен как счётчик с периодом 0xFFFF (65535 тактов). Частота счётчика:  
`f_timer = 72 МГц / (71 + 1) = 1 МГц`, период тика = 1 мкс.

Максимальное измеряемое время до переполнения – 65535 мкс ≈ 65,5 мс, что перекрывает период PPM‑кадра (до 27 мс).

## 2. Анализ исходного кода

Основные файлы: `timer.ino` (настройка TIM2) и `Tx_and_Rx.ino` (обработчик прерывания PPM).

### 2.1. Настройка таймера TIM2 (timer.ino)

```c
void timer_setup() {
    TIMER2_BASE->CR1 = TIMER_CR1_CEN;   // включить счётчик
    TIMER2_BASE->PSC = 71;              // предделитель: 72МГц/(71+1)=1МГц
    TIMER2_BASE->ARR = 0xFFFF;          // счёт до 65535

    // Канал 1 – вход захвата TI1
    TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
    TIMER2_BASE->CCER = TIMER_CCER_CC1E;      // разрешить захват
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;    // захват по нарастающему фронту
    TIMER2_BASE->DIER = TIMER_DIER_CC1IE;     // прерывание по захвату
    Timer2.attachInterrupt(1, handler_channel_1);
}
```

### 2.2. Обработчик прерывания PPM (Tx_and_Rx.ino)

```c
volatile int32_t measured_time, measured_time_start;
volatile uint8_t channel_select_counter = 0;

void handler_channel_1() {
    // 1) Вычисляем длительность импульса
    measured_time = TIMER2_BASE->CCR1 - measured_time_start;
    // 2) Коррекция переполнения счётчика
    if (measured_time < 0) measured_time += 0xFFFF;
    // 3) Запоминаем текущее время
    measured_time_start = TIMER2_BASE->CCR1;

    // 4) Определяем тип импульса
    if (measured_time > 3000) {
        channel_select_counter = 0;        // синхроимпульс -> начало кадра
    } else if (measured_time >= 800 && measured_time <= 2200) {
        channel_select_counter++;
        switch(channel_select_counter) {
            case 1: channel_1 = measured_time; break;  // Roll
            case 2: channel_2 = measured_time; break;  // Pitch
            case 3: channel_3 = measured_time; break;  // Throttle
            case 4: channel_4 = measured_time; break;  // Yaw
            case 5: channel_5 = measured_time; break;  // Flight Mode
            // ... до channel_10
        }
    }
}
```

**Пояснения:** переменные объявлены как `volatile` – это запрещает компилятору кэшировать их значения. Обработчик должен быть коротким, чтобы не блокировать другие прерывания и главный цикл.

## 3. Структурные схемы и блок-схемы

### Рисунок 1.1 – Временная диаграмма PPM-сигнала

<p align="center">
  <svg width="800" height="180" viewBox="0 0 800 180" xmlns="http://www.w3.org/2000/svg">
    <line x1="30" y1="100" x2="770" y2="100" stroke="black" stroke-width="1.5" marker-end="url(#arrow)"/>
    <defs><marker id="arrow" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto"><polygon points="0 0, 10 3.5, 0 7" fill="black"/></marker></defs>
    <rect x="30" y="50" width="180" height="40" fill="none" stroke="black" stroke-width="2"/>
    <text x="120" y="45" text-anchor="middle" font-size="12">синхроимп.</text>
    <text x="120" y="80" text-anchor="middle" font-size="11">&gt;3000 мкс</text>
    <rect x="220" y="50" width="75" height="40" fill="none" stroke="black" stroke-width="2"/>
    <text x="257" y="45" text-anchor="middle" font-size="12">Канал 1</text>
    <text x="257" y="80" text-anchor="middle" font-size="11">1500 мкс</text>
    <rect x="305" y="50" width="60" height="40" fill="none" stroke="black" stroke-width="2"/>
    <text x="335" y="45" text-anchor="middle" font-size="12">Канал 2</text>
    <text x="335" y="80" text-anchor="middle" font-size="11">1200 мкс</text>
    <rect x="375" y="50" width="90" height="40" fill="none" stroke="black" stroke-width="2"/>
    <text x="420" y="45" text-anchor="middle" font-size="12">Канал 3</text>
    <text x="420" y="80" text-anchor="middle" font-size="11">1800 мкс</text>
    <rect x="475" y="50" width="50" height="40" fill="none" stroke="black" stroke-width="2"/>
    <text x="500" y="45" text-anchor="middle" font-size="12">Канал 4</text>
    <text x="500" y="80" text-anchor="middle" font-size="11">1000 мкс</text>
    <line x1="30" y1="120" x2="770" y2="120" stroke="gray" stroke-dasharray="4,4"/>
    <text x="400" y="140" text-anchor="middle" font-size="13" font-style="italic">период кадра (24 мс для 10 каналов)</text>
    <line x1="30" y1="115" x2="30" y2="125" stroke="gray"/>
    <line x1="770" y1="115" x2="770" y2="125" stroke="gray"/>
  </svg>
</p>

### Рисунок 1.2 – Блок-схема обработчика прерывания PPM

<p align="center">
  <svg width="750" height="650" viewBox="0 0 750 650" xmlns="http://www.w3.org/2000/svg">
    <rect x="275" y="10" width="200" height="40" rx="10" fill="lightgreen" stroke="black" stroke-width="1.5"/>
    <text x="375" y="35" text-anchor="middle" font-size="13">Прерывание по фронту PA0</text>
    <polygon points="375,55 375,75" fill="black"/>
    <rect x="250" y="80" width="250" height="50" fill="white" stroke="black"/>
    <text x="375" y="100" text-anchor="middle" font-size="12">measured_time = CCR1 -</text>
    <text x="375" y="118" text-anchor="middle" font-size="12">measured_time_start</text>
    <polygon points="375,135 375,155" fill="black"/>
    <polygon points="375,160 415,195 375,230 335,195" fill="white" stroke="black"/>
    <text x="375" y="200" text-anchor="middle" font-size="12">measured_time</text>
    <text x="375" y="215" text-anchor="middle" font-size="12">&lt; 0 ?</text>
    <line x1="335" y1="195" x2="180" y2="195" stroke="black"/>
    <polygon points="180,190 170,195 180,200" fill="black"/>
    <rect x="60" y="220" width="220" height="40" fill="white" stroke="black"/>
    <text x="170" y="245" text-anchor="middle" font-size="12">measured_time += 0xFFFF</text>
    <line x1="170" y1="260" x2="170" y2="300" stroke="black"/>
    <polygon points="165,300 170,310 175,300" fill="black"/>
    <line x1="415" y1="195" x2="600" y2="195" stroke="black"/>
    <line x1="600" y1="195" x2="600" y2="290" stroke="black"/>
    <line x1="170" y1="310" x2="600" y2="310" stroke="black" stroke-dasharray="5,3"/>
    <rect x="300" y="320" width="250" height="40" fill="white" stroke="black"/>
    <text x="425" y="345" text-anchor="middle" font-size="12">measured_time_start = CCR1</text>
    <polygon points="425,365 425,385" fill="black"/>
    <polygon points="425,390 465,425 425,460 385,425" fill="white" stroke="black"/>
    <text x="425" y="430" text-anchor="middle" font-size="12">measured_time</text>
    <text x="425" y="445" text-anchor="middle" font-size="12">&gt; 3000 ?</text>
    <line x1="385" y1="425" x2="200" y2="425" stroke="black"/>
    <polygon points="200,420 190,425 200,430" fill="black"/>
    <rect x="90" y="450" width="210" height="40" fill="white" stroke="black"/>
    <text x="195" y="475" text-anchor="middle" font-size="12">channel_select_counter = 0</text>
    <line x1="195" y1="490" x2="195" y2="580" stroke="black"/>
    <line x1="465" y1="425" x2="560" y2="425" stroke="black"/>
    <polygon points="560,420 570,425 560,430" fill="black"/>
    <polygon points="570,440 610,475 570,510 530,475" fill="white" stroke="black"/>
    <text x="570" y="465" text-anchor="middle" font-size="11">800 ≤ t ≤ 2200 ?</text>
    <line x1="570" y1="510" x2="570" y2="540" stroke="black"/>
    <polygon points="565,540 570,550 575,540" fill="black"/>
    <rect x="460" y="555" width="220" height="40" fill="white" stroke="black"/>
    <text x="570" y="575" text-anchor="middle" font-size="12">channel_select_counter++</text>
    <polygon points="570,600 570,620" fill="black"/>
    <rect x="440" y="625" width="260" height="50" fill="white" stroke="black"/>
    <text x="570" y="645" text-anchor="middle" font-size="12">switch → channel_1 ... channel_10</text>
    <line x1="195" y1="620" x2="195" y2="650" stroke="black"/>
    <line x1="195" y1="650" x2="440" y2="650" stroke="black"/>
    <rect x="330" y="600" width="100" height="35" rx="10" fill="lightcoral" stroke="black"/>
    <text x="380" y="622" text-anchor="middle" font-size="13">Конец ISR</text>
  </svg>
</p>

### Рисунок 1.3 – Схема подключения RC-приёмника к STM32F103

<p align="center">
  <svg width="500" height="250" viewBox="0 0 500 250" xmlns="http://www.w3.org/2000/svg">
    <rect x="50" y="50" width="140" height="80" fill="#e0f0ff" stroke="black" stroke-width="2"/>
    <text x="120" y="80" text-anchor="middle" font-size="14" font-weight="bold">RC-приёмник</text>
    <text x="120" y="100" text-anchor="middle" font-size="12">(PPM out)</text>
    <text x="120" y="120" text-anchor="middle" font-size="11">GND</text>
    <rect x="300" y="50" width="150" height="100" fill="#ffe0e0" stroke="black" stroke-width="2"/>
    <text x="375" y="80" text-anchor="middle" font-size="14" font-weight="bold">STM32F103</text>
    <text x="375" y="105" text-anchor="middle" font-size="12">PA0 (TIM2 CH1)</text>
    <text x="375" y="130" text-anchor="middle" font-size="12">GND</text>
    <line x1="190" y1="80" x2="300" y2="80" stroke="black" stroke-width="2"/>
    <polygon points="295,75 305,80 295,85" fill="black"/>
    <text x="245" y="70" text-anchor="middle" font-size="12">PPM сигнал</text>
    <line x1="190" y1="120" x2="300" y2="120" stroke="black" stroke-width="2" stroke-dasharray="4,2"/>
    <polygon points="295,115 305,120 295,125" fill="black"/>
    <text x="245" y="135" text-anchor="middle" font-size="12">общая земля</text>
    <text x="250" y="200" text-anchor="middle" font-size="11" fill="gray">(сигнальный уровень – 3.3 В совместим)</text>
  </svg>
</p>

## 4. Расчётная часть

### Задание 11.1. Форматы RC-сигналов и временные параметры

**б) Период PPM-кадра для 10 каналов:**  
`T_frame = N_ch · t_max + t_sync = 10·2000 + 4000 = 24000 мкс = 24 мс`.

**в) Частота обновления каналов:**  
`f_RC = 1 / T_frame = 1 / 0.024 ≈ 41,67 Гц`.

### Задание 11.2. Назначение каналов

| Канал | Переменная   | Функция          | Диапазон, мкс |
|-------|--------------|------------------|---------------|
| 1     | channel_1    | Roll (крен)      | 1000–2000     |
| 2     | channel_2    | Pitch (тангаж)   | 1000–2000     |
| 3     | channel_3    | Throttle (газ)   | 1000–2000     |
| 4     | channel_4    | Yaw (рыскание)   | 1000–2000     |
| 5     | channel_5    | Flight mode      | <1300 / 1300–1700 / >1700 |
| 6–10  | channel_6–10 | Вспомогательные  | 1000–2000     |

### Задание 11.3. Анализ обработчика, переполнение, volatile

**Переполнение счётчика:** Пример: старый счётчик = 65000, новый = 1000. Разность = -64000, после +65535 = 1535 мкс – верная длительность.

**Ключевое слово volatile:** запрещает оптимизацию, гарантирует чтение актуального значения из памяти.

**Задержка от движения стика до обновления channel_1:** в худшем случае – почти целый кадр (24 мс) плюс время до конкретного канала (до 2000 мкс). Итого ~26 мс.

### Задание 11.4. Fail‑Safe

В коде при отсутствии синхроимпульсов >1 с газ (channel_3) принудительно устанавливается в 1000 мкс (останов моторов). Предложено улучшение – активация RTH при потере сигнала.

### Задание 11.5. Цифровые протоколы RC

| Протокол | Тип        | Латентность | Шифрование |
|----------|------------|-------------|-------------|
| PPM      | Аналоговый | 15–27 мс    | Нет         |
| SBUS     | Цифровой   | ~7 мс       | Нет         |
| CRSF     | Цифровой   | <2 мс       | Нет         |
| ELRS     | Цифровой   | <1 мс       | Да (AES)    |

## 5. Предложение по улучшению

В текущей реализации Fail‑Safe при потере сигнала PPM просто останавливает моторы (устанавливает channel_3 = 1000 мкс). Это опасно при полёте над водой, лесом или городской застройкой – дрон падает.

**Предлагаемое улучшение:** вместо остановки моторов активировать автоматический режим **Return‑to‑Home (RTH)**, если есть валидные GPS‑координаты домашней точки. Алгоритм:
1. При отсутствии новых кадров PPM в течение 1 секунды – переключить flight_mode в режим 4 (RTH).
2. Дрон набирает безопасную высоту 15 метров (если текущая высота ниже).
3. По GPS‑координатам рассчитывается курс и расстояние до точки взлёта, дрон летит к ней.
4. После прибытия в радиус 2 м – плавное снижение со скоростью 0,5 м/с и посадка.

Если GPS отсутствует (нет фикса или мало спутников) – выполняется **плавная посадка** на месте с помощью лидара/барометра (снижение со скоростью 0,3 м/с до касания земли).

Сложность реализации – средняя. Потребуется модифицировать модули `RTH.ino`, `Telemetry.ino` и `PID.ino`. Такое улучшение соответствует стандартам безопасности DO‑178C (уровень DAL‑C) и нормам ИКАО для БПЛА массой более 250 г.

## Заключение

В ходе выполнения курсовой работы по варианту 11 «Система дистанционного управления» были решены следующие задачи:
- Изучены теоретические основы форматов сигналов PWM и PPM, принципы работы таймеров STM32 в режиме Input Capture.
- Выполнен детальный анализ исходного кода (`timer.ino`, `Tx_and_Rx.ino`) учебного полётного контроллера. Показано, как обработчик прерывания декодирует PPM-кадр и заполняет переменные `channel_1…channel_10`.
- Разработаны три наглядные схемы: временная диаграмма PPM, блок-схема обработчика и схема подключения RC-приёмника к STM32F103.
- Выполнены расчёты временных параметров: период кадра для 10 каналов – 24 мс, частота обновления – ≈42 Гц. Оценена задержка от движения стика до изменения переменной (около 26 мс).
- Рассмотрен механизм Fail‑Safe, выявлен его главный недостаток – немедленная остановка моторов. Предложено улучшение с активацией RTH или плавной посадкой, что значительно повышает безопасность полёта.

Полученные знания и навыки могут быть применены при разработке реальных систем дистанционного управления для беспилотных авиационных систем, а также при написании собственного полётного контроллера.

## Список литературы

1. STM32F103 Reference Manual RM0008. – STMicroelectronics, 2021. – 1136 с.
2. Куприянов М.С., Матюшкин Б.Д. Цифровая обработка сигналов. – СПб.: Политехника, 1999. – 286 с.
3. Документация ArduPilot: RC input and failsafe handling. – URL: https://ardupilot.org/copter/docs/radio-failsafe.html
4. Документация PX4 Autopilot: Remote Control. – URL: https://docs.px4.io/main/en/advanced_config/radio.html
5. Постановление Правительства РФ от 11 марта 2010 г. № 138 «Об утверждении Федеральных правил использования воздушного пространства Российской Федерации» (с изменениями для БПЛА).
6. Mahony R., Hamel T., Pflimlin J.-M. Nonlinear complementary filters on the special orthogonal group. – IEEE Transactions on Automatic Control, 2008, vol. 53, no. 5, pp. 1203–1218.
```
