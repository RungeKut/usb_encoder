### USER
HID Descriptor Tool как пользоваться утилитой? и по какому приципу нужно составлять дискриптор устройства?

### ASSISTANT
**HID Descriptor Tool** — это утилита от Microsoft, предназначенная для создания, анализа и отладки **HID-дескрипторов** (Human Interface Device descriptors). Она особенно полезна разработчикам встраиваемых устройств, которые реализуют USB HID-устройства (например, клавиатуры, мыши, джойстики, кастомные панели управления и т.п.).

---

## 🔧 Как пользоваться HID Descriptor Tool

### 1. **Скачивание и установка**
- Утилита входит в состав **Windows Driver Kit (WDK)** или доступна отдельно как часть **HID Tools** от Microsoft.
- Актуальная версия: **HID Descriptor Tool v2.5** (часто распространяется как `HIDTool.exe`).
- Скачать можно с официального сайта Microsoft (иногда через архивы WDK или в составе старых SDK).

> ⚠️ Утилита довольно старая (последнее обновление — ~2007 г.), но до сих пор актуальна для работы с HID.

---

### 2. **Основной интерфейс**
После запуска вы видите:
- **Дерево дескриптора** слева (в виде иерархии тегов).
- **Панель редактирования** справа.
- Кнопки: **Add Item**, **Delete Item**, **Parse**, **Generate**, **Validate** и т.д.

---

### 3. **Создание дескриптора**
#### Шаги:
1. Нажмите **File → New** (или начните с шаблона).
2. Добавьте корневой элемент: **Main → Collection (Application)**.
3. Внутри коллекции добавьте:
   - **Global Items** (например, Usage Page, Logical Minimum/Maximum).
   - **Local Items** (например, Usage).
   - **Main Items** (Input, Output, Feature).
4. Заполните значения согласно спецификации HID.

> Пример: создание простой кнопки (Usage = Button 1, Input = Data,Var,Abs).

---

### 4. **Генерация и экспорт**
- Нажмите **Generate** → утилита создаст байтовый массив дескриптора.
- Можно скопировать его в C-код (массив `const uint8_t hid_report_descriptor[] = {...};`).
- Также можно сохранить как `.txt` или `.bin`.

---

### 5. **Анализ существующего дескриптора**
- Скопируйте байты дескриптора (например, из дампа USB-устройства).
- Вставьте в поле **Raw Descriptor**.
- Нажмите **Parse** → утилита построит дерево и покажет структуру.

---

## 📜 Принципы составления HID-дескриптора

HID-дескриптор описывает **формат отчётов (reports)**, которые устройство отправляет/принимает. Он состоит из **элементов (items)**, разделённых на три категории:

| Категория      | Назначение |
|----------------|-----------|
| **Main Items** | Определяют структуру данных: `Input`, `Output`, `Feature`, `Collection`, `End Collection`. |
| **Global Items** | Действуют до конца коллекции: `Usage Page`, `Logical Min/Max`, `Report Size`, `Report Count`, `Report ID`. |
| **Local Items** | Действуют только до следующего Main Item: `Usage`, `Usage Minimum/Maximum`. |

---

### 🔑 Основные правила:
1. **Начинайте с `Usage Page`** — определяет тип устройства (например, `Generic Desktop` = 0x01, `Button` = 0x09).
2. **Укажите `Usage`** — конкретная функция (например, `Mouse` = 0x02, `X` = 0x30, `Button 1` = 0x01).
3. **Определите диапазон значений**: `Logical Minimum/Maximum`.
4. **Задайте размер и количество полей**: `Report Size` (бит), `Report Count` (сколько таких полей).
5. **Используйте `Input`/`Output`/`Feature`**:
   - `Input` — данные от устройства к хосту (например, координаты мыши).
   - `Output` — данные от хоста к устройству (например, светодиоды клавиатуры).
   - `Feature` — двунаправленные настройки (редко).
6. **Группируйте логически связанные элементы в `Collection`** (Application, Logical, Physical).

---

### 📌 Пример: простой дескриптор кнопки

```c
0x05, 0x01,        // Usage Page (Generic Desktop)
0x09, 0x02,        // Usage (Mouse)
0xA1, 0x01,        // Collection (Application)
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x05, 0x09,        //     Usage Page (Button)
0x19, 0x01,        //     Usage Minimum (Button 1)
0x29, 0x03,        //     Usage Maximum (Button 3)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x01,        //     Logical Maximum (1)
0x95, 0x03,        //     Report Count (3)
0x75, 0x01,        //     Report Size (1)
0x81, 0x02,        //     Input (Data,Var,Abs)
0x95, 0x01,        //     Report Count (1)
0x75, 0x05,        //     Report Size (5)
0x81, 0x01,        //     Input (Cnst,Ary,Abs) — padding
0xC0,              //   End Collection
0xC0               // End Collection
```

---

## 📚 Полезные ресурсы
- **[HID Specification 1.11](https://www.usb.org/document-library/device-class-definition-hid-111)** — официальный документ от USB-IF.
- **HID Usage Tables** — таблицы значений Usage Page и Usage.
- **USBlyzer / Wireshark** — для захвата и анализа реальных HID-дескрипторов.
- **TinyUSB / LUFA / Zephyr** — примеры дескрипторов в open-source проектах.

---

## 💡 Советы
- Всегда **валидируйте** дескриптор в HID Descriptor Tool.
- Следите за **выравниванием битов** — если `Report Size × Report Count` не кратно 8, может потребоваться padding.
- Используйте **Report ID**, если у вас несколько типов отчётов.

---

Если хочешь — могу помочь составить дескриптор под твоё конкретное устройство (укажи: тип, сколько кнопок/осей/светодиодов и т.п.).