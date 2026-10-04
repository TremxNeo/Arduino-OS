# PCuino AppSDK

PCuino uses a drop-in application model for Arduino. The operating system provides the core API; applications self-register through `AppRegistrar`.

## Project layout

```text
PCuino/
├── PCuino.ino
├── config.h
├── core_api.h
├── README.md
├── src/
│   └── apps/
│       ├── chat/
│       │   └── app.cpp
│       ├── ftp/
│       │   └── app.cpp
│       ├── viewer/
│       │   └── app.cpp
│       └── <your-app>/
│           └── app.cpp
└── toolkit/
    └── PCuino_AppSDK.html
```

## Recommended Terminal Software

The **Audio FTP OS** relies on ANSI escape sequences for its interactive shell, status bars, progress indicators, and color-coded output.

For the best experience, use **Tera Term** on Windows or another fully ANSI-compatible serial terminal emulator.

### Recommended Tera Term Settings

Tera Term communicates with the Arduino through its USB serial interface.

Configure the serial connection as follows:

- **Baud rate:** `115200`
- **Data:** `8 bit`
- **Parity:** `None`
- **Stop:** `1 bit`
- **Flow control:** `None`

This is the standard:

```text
115200 8N1
```

### Terminal Setup

In Tera Term:

**Setup → Terminal**

Recommended settings:

- **Terminal size:** `80 × 24` or your preferred window size
- **Receive:** `AUTO` or `CR+LF`
- **Transmit:** `CR` or `CR+LF`
- **Local echo:** `Off`
- **ANSI color:** Enabled

The PCuino shell handles command echoing and ANSI display control itself.

### Keyboard

Configure the Backspace key to send `DEL` (`0x7F`) or `BS`, depending on the input handling expected by the current firmware.

### Important distinction

The `115200 8N1` configuration applies to:

```text
PC
 │
 │ USB Serial
 │ 115200 8N1
 │
 ▼
Arduino PCuino
```

It does **not** describe the PCuino node-to-node communication system.

The node-to-node link uses the custom PCuino tone modem described below.

---

# Wiring Two PCuino Nodes

Two PCuino nodes communicate through the custom **tone modem**.

**This is not a UART TX/RX connection.**

The modem uses the Arduino's Timer1 hardware:

| Arduino pin | Function | Timer1 function |
|---|---|---|
| **D9** | Modem transmit | `OC1A` |
| **D8** | Modem receive | `ICP1` |

The firmware reserves these pins for the modem.

## Node-to-node wiring

Connect the two nodes crossed:

```text
             PCuino Node A                 PCuino Node B

             Arduino Uno                  Arduino Uno
           ┌──────────────┐             ┌──────────────┐
           │              │             │              │
           │ D9 / TX  ────┼─────────────┼───> D8 / RX  │
           │              │             │              │
           │ D8 / RX  <───┼─────────────┼──── D9 / TX  │
           │              │             │              │
           └──────────────┘             └──────────────┘
```

In simplified form:

```text
Node A D9  →  Node B D8
Node A D8  ←  Node B D9
```

The important point is that these labels describe the **PCuino modem direction**, not UART.

### D9 — modem output

D9 is the Timer1 `OC1A` output.

It generates the modem waveform used to represent:

- characters
- spaces
- repeat operations
- layer changes
- protocol operations
- transfer operations

### D8 — modem input

D8 is the Timer1 `ICP1` input-capture pin.

The receiving Arduino measures the incoming waveform using Timer1 input capture and determines which tone was transmitted.

## Do not wire the modem as ordinary UART

Do **not** substitute:

```text
D1 / TX
D0 / RX
```

for the modem connection.

The PCuino modem specifically uses:

```text
D9 → modem output
D8 → modem input
```

because these pins correspond to the Timer1 hardware used by the modem.

## USB serial and modem connections are separate

A typical two-node setup is:

```text
                  USB SERIAL
              115200 8N1
                   │
                   ▼
              ┌───────────┐
              │ PC /      │
              │ Tera Term │
              └─────┬─────┘
                    │ USB
                    ▼
             ┌──────────────┐
             │  PCuino A    │
             │              │
             │ D9 ──────────┼──────────> D8
             │ D8 <─────────┼────────── D9
             └──────────────┘
                    ▲
                    │
             ┌──────┴───────┐
             │  PCuino B    │
             └──────────────┘
```

The PC talks to the Arduino using normal USB serial.

The two Arduinos communicate using the custom tone modem.

---

# PCuino Tone Modem

PCuino uses a custom frequency-based modem for node-to-node communication.

Instead of sending protocol information as ordinary UART bytes, the transmitter converts characters and protocol operations into a sequence of tones.

The receiver measures those tones and reconstructs the original protocol information.

The overall process is:

```text
Logical protocol frame
        │
        ▼
Character / protocol tokenization
        │
        ▼
Layer selection
        │
        ▼
Frequency selection
        │
        ▼
Tone generation on D9
        │
        ▼
PCuino modem link
        │
        ▼
Tone measurement on D8
        │
        ▼
Timer1 input capture
        │
        ▼
Frequency decoding
        │
        ▼
Character / protocol reconstruction
        │
        ▼
CRC verification
        │
        ▼
Protocol response
```

This makes the modem fundamentally different from a conventional UART connection.

---

# Tone / Character Table

The modem uses frequency ranges to represent characters and protocol operations.

The character system is organized into **layers**.

A layer-selection tone tells the receiver which character table is currently active. The following character tone is then interpreted according to that layer.

The modem therefore does not need a unique control frequency for every possible character.

Instead:

```text
Layer
  +
Character position
  =
Character
```

## Basic modem tones

The modem reserves specific frequencies for control operations.

The principal control tones are:

| Function | Frequency |
|---|---:|
| Preamble | `3000 Hz` |
| Space | `3060 Hz` |
| Repeat previous character | `3120 Hz` |

Layer selection begins at:

```text
3180 Hz
```

with a `60 Hz` step.

Conceptually:

| Layer | Frequency |
|---:|---:|
| 0 | `3180 Hz` |
| 1 | `3240 Hz` |
| 2 | `3300 Hz` |
| 3 | `3360 Hz` |
| 4 | `3420 Hz` |
| 5 | `3480 Hz` |
| 6 | `3540 Hz` |
| 7 | `3600 Hz` |
| 8 | `3660 Hz` |
| 9 | `3720 Hz` |

Character/data frequencies begin at approximately:

```text
3800 Hz
```

with a frequency increment of:

```text
123 Hz
```

per character position.

---

# Character Layers

The printable character set is divided into groups of 13 characters.

The character mapping follows the layer/position model:

```text
ASCII character
       │
       ├── layer
       │
       └── position within layer
                    │
                    ▼
             data-tone frequency
```

The printable ASCII mapping is organized conceptually as:

```text
Layer 0:
! " # $ % & ' ( ) * + , -

Layer 1:
. / 0 1 2 3 4 5 6 7 8 9 :

Layer 2:
; < = > ? @ A B C D E F G

Layer 3:
H I J K L M N O P Q R S T

Layer 4:
U V W X Y Z [ \ ] ^ _ ` a

Layer 5:
b c d e f g h i j k l m n

Layer 6:
o p q r s t u v w x y z {
```

The layer-selection tone is transmitted before a character when the required character belongs to a different layer.

---

# Layer Switching

The transmitter maintains the currently selected layer.

If the next character belongs to another layer, it first sends the corresponding layer-selection tone.

For example:

```text
[select layer]
[character]
[character]
[character]
```

The receiver changes its active layer when it detects the layer-selection tone.

This avoids transmitting a complete character identifier for every character.

The receiver only needs:

```text
current layer
+
character tone position
```

to reconstruct the character.

---

# Space Encoding

A space has its own dedicated tone:

```text
3060 Hz
```

This means the transmitter does not need to switch to a character layer simply to transmit a space.

A space is therefore represented directly by the reserved SPACE tone.

---

# Repeat Encoding

The modem also has a dedicated repeat tone:

```text
3120 Hz
```

The repeat tone means:

> transmit the same character as the previously decoded character.

For example, a repeated sequence such as:

```text
AAAAAA
```

can use:

```text
A
REPEAT
REPEAT
REPEAT
REPEAT
REPEAT
```

rather than transmitting the complete character encoding six times.

This is particularly useful for repetitive data.

---

# Preamble

A transmission begins with the modem preamble:

```text
3000 Hz
```

The preamble establishes the beginning of a new modem frame and allows the receiver to reset relevant decoding state.

Conceptually:

```text
PREAMBLE
   ↓
initial modem state
   ↓
layer / token decoding
   ↓
payload
```

---

# Protocol Operation Encoding

The modem is not limited to individual characters.

Frequently used protocol operations can be represented directly as compact modem tokens.

This avoids transmitting a long protocol word character-by-character.

Examples include operations such as:

```text
HELLO
HACK
ACK
NAK
PING
LOCK
LKOK
LKNO
UNLK
```

and file-management operations such as:

```text
LS
LSD
CAT
CATD
GET
GETD
RM
PUT
```

Transfer-specific operations include:

```text
XBEG
XDAT
XABT
XGO
```

The exact operation-to-tone assignments are part of the modem/protocol table.

The important design principle is:

```text
Common protocol operation
        ↓
single compact modem token
```

rather than:

```text
Common protocol operation
        ↓
individual ASCII characters
        ↓
individual character tones
```

This substantially reduces the amount of modem signaling required for frequently used commands.

---

# Protocol Dictionary

The protocol dictionary contains the logical operations understood by the PCuino system.

Examples include:

```text
HELLO
HACK
ACK
NAK
PING
LOCK
LKOK
LKNO
UNLK

LS:eeprom
LSD:eeprom
CAT:eeprom
CATD
GET:eeprom
GETD
RM:eeprom
PUT:eeprom

OK:Saved
OK:Deleted
OK:EEPROM formatted

ERR:NOLOCK
ERR:DENIED
ERR:NOTFOUND
ERR:BADNAME
ERR:TOOLONG
ERR:FULL
ERR

XBEG
XDAT
XABT
XGO

APP
```

The modem can represent these operations using compact protocol tokens rather than sending every letter independently.

---

# Transmission Format

The logical protocol frame is separate from the physical tone encoding.

A frame follows the general structure:

```text
<sequence> <payload>#<CRC16>
```

For example:

```text
12 ACK#A53F
```

The logical frame contains:

1. Sequence information
2. Protocol payload
3. CRC separator
4. CRC16 value

The CRC is calculated over the logical frame before the CRC field is appended.

The CRC is represented as four hexadecimal characters:

```text
#XXXX
```

The modem then converts the resulting logical frame into tones.

Therefore:

```text
Logical frame
      ↓
CRC calculation
      ↓
Protocol/token encoding
      ↓
Layer selection
      ↓
Tone sequence
```

The receiver performs the reverse operation.

---

# Example Transmission

Consider a logical transfer frame:

```text
7 XDAT 0012:Hello#3A9F
```

The transmitter does not simply send:

```text
7
space
X
D
A
T
...
```

as UART characters.

Instead, it interprets the protocol and converts its contents into modem tokens.

Conceptually:

```text
7 XDAT 0012:Hello#3A9F
          │
          ▼
protocol/token encoding
          │
          ├── character tones
          ├── layer changes
          ├── SPACE tone
          └── XDAT protocol token
          │
          ▼
physical tone sequence
```

The receiver measures the sequence and reconstructs the logical frame.

---

# Tone Timing

The modem transmits tones as waveform cycles rather than as ordinary UART bit periods.

The transmitter generates the required frequency using Timer1 hardware.

The receiver determines the frequency by measuring the incoming waveform using Timer1 input capture.

The modem therefore operates according to:

```text
frequency
+
tone duration
+
inter-tone timing
```

rather than:

```text
UART baud rate
+
start bit
+
data bits
+
stop bit
```

The tone timing and inter-tone delays are part of the modem implementation and should be kept synchronized between transmitter and receiver.

---

# Timer1 Modem Hardware

The modem uses the Arduino's Timer1 hardware.

The two modem pins are:

```text
D9 = OC1A
D8 = ICP1
```

### Transmit path

```text
Timer1
  │
  └── OC1A
       │
       ▼
      D9
       │
       ▼
   modem output
```

### Receive path

```text
modem input
      │
      ▼
     D8
      │
      ▼
 Timer1 ICP1
      │
      ▼
input-capture measurement
      │
      ▼
frequency decoding
```

This is why the modem connection cannot be replaced by ordinary UART wiring.

---

# File Transfer Protocol

PCuino's FTP system uses the tone modem for file transfer.

Transfer operations include:

```text
XBEG
XDAT
XGO
XABT
```

The transfer protocol separates control information from the actual file-data payload.

`XDAT` is used for data transfer.

The modem and higher-level protocol provide acknowledgement and error handling so that a transmission failure can be detected rather than silently accepted.

---

# ACK / NAK and Reliability

The tone modem is the physical signaling layer.

Above it, the PCuino protocol provides communication control.

A successfully received frame can produce:

```text
ACK
```

while an invalid or rejected frame can produce:

```text
NAK
```

This provides a second layer of reliability above simple tone detection.

The receiver therefore does not merely need to detect a valid frequency. It must also reconstruct the logical frame and verify its integrity.

The overall receive path is:

```text
Tone detected
      ↓
Frequency identified
      ↓
Token reconstructed
      ↓
Logical frame reconstructed
      ↓
CRC checked
      ↓
Protocol operation processed
      ↓
ACK / NAK
```

---

# Important Serial Distinction

There are two completely different communication systems in PCuino.

## PC connection

The PC/Arduino connection uses:

```text
USB Serial
115200 baud
8 data bits
No parity
1 stop bit
No flow control
```

Tera Term is used for this interface.

```text
PC
 │
 │ USB
 │
 ▼
Arduino
```

## Arduino-to-Arduino connection

The PCuino node link uses:

```text
D9 / OC1A → tone transmission
D8 / ICP1 → tone reception
```

It uses the custom frequency-based modem.

```text
Arduino A
   │
   │ D9 tone output
   ▼
  modem
   │
   ▼
Arduino B
   │
   │ D8 tone input
   ▼
Timer1 input capture
```

Therefore:

> **115200 baud applies to Tera Term/USB serial. It does not describe the PCuino modem link.**

---

# Creating an App

Use **PCuino AppSDK** to build the app visually.

The SDK generates exactly one source file:

```text
app.cpp
```

Put it in:

```text
src/apps/<name>/app.cpp
```

Because the Arduino build compiles source files under `src/apps/`, no central app list needs to be edited.

---

# App Registration

Every app defines its `AppHooks` and creates a static registrar:

```cpp
static const AppRegistrar myapp_auto_register(&myapp_hooks);
```

The presence of the app's `.cpp` is its installation switch.

There is intentionally **no**:

- `APP_CHAT`, `APP_FTP`, `APP_VIEWER`, or other app-selection flag
- `register_all_apps()`
- `apps_bundle.cpp`
- central application list
- generated `app.h`
- manual registration step

---

# Core API Location

`core_api.h` is at the **project root**.

An app generated by AppSDK therefore includes it with:

```cpp
#include "../../../core_api.h"
```

from:

```text
src/apps/<name>/app.cpp
```

---

# AppSDK Block Model

Event blocks are the top-level handlers:

- When App Opens
- When User Enters Line
- When Message Received
- When Send Finishes
- When Disconnected

Actions are placed inside an event.

Logic blocks such as:

- **If Online**
- **If Message Equals**

can contain further actions.

AppSDK preserves this nesting when generating C++ instead of flattening all actions into every event.

`If Message Equals` is intended for the **Message Received** event because it tests the received `data` buffer.

---

# Built-in Applications

The clean build includes:

- **Chat** — application messaging UI
- **FTP** — file transfer/filesystem UI
- **Viewer** — file viewing UI

Additional apps can be added by dropping their generated folder into:

```text
src/apps/
```

---

# App IDs

Each app has a one-character printable Wire/App ID.

IDs must:

- be unique among installed applications
- be printable
- not be `:`

The application ID is used by the PCuino application/wire system to identify the application.

---

# Updating an Existing App

Apps created with an older App Maker may contain:

```text
app.h
*_register()
APP_* flags
old core_api.h includes
```

Regenerate the app with the current **PCuino AppSDK** and replace its `app.cpp`.

The current SDK:

- generates only `app.cpp`
- uses `../../../core_api.h`
- self-registers with `AppRegistrar`

---

# Removing an App

Delete its app folder from:

```text
src/apps/
```

No core source or configuration file needs to be changed.

The presence of the application's `.cpp` file is its installation switch.

---

# Important Arduino Note

Do not keep obsolete copies of:

```text
apps_bundle.cpp
old app_*.h files
legacy APP_* controlled apps
```

in the sketch tree.

Arduino may compile `.cpp` files that remain in the project, even if they are not part of the intended architecture.

Therefore, when migrating to the current AppSDK architecture, remove obsolete application implementations rather than simply leaving them in the project.

---

# Import / Decompile an Existing App

The PCuino AppSDK can import an existing `app.cpp` with **Import app.cpp**.

It recognizes AppSDK-generated:

- event patterns
- action patterns
- logic patterns

and reconstructs them as blocks.

C++ that has no direct AppSDK block is preserved as a:

```text
Custom C++
```

block rather than being discarded.

Global declarations needed by an imported app are preserved as:

```text
Custom Global C++
```

This supports inspecting built-in or hand-written apps such as Viewer without pretending unsupported C++ is equivalent to a native block.

AppSDK-generated apps can therefore round-trip between:

```text
Visual blocks
     ↕
app.cpp
```

much more closely.

---

# Architecture Summary

PCuino consists of several distinct layers:

```text
┌─────────────────────────────────────────┐
│              PCuino Apps                │
│                                         │
│     Chat   FTP   Viewer   Custom Apps   │
└───────────────────┬─────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────┐
│               Core API                  │
│              core_api.h                 │
└───────────────────┬─────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────┐
│          PCuino Protocol Layer          │
│                                         │
│     Commands / Apps / FTP / ACK / CRC   │
└───────────────────┬─────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────┐
│          Tone Character System          │
│                                         │
│   Layers / Character tones / Controls   │
└───────────────────┬─────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────┐
│             Tone Modem                  │
│                                         │
│       D9 / OC1A     D8 / ICP1           │
│        TX tone       RX capture         │
└─────────────────────────────────────────┘
```

The PCuino system therefore has a clear separation between:

1. **Applications**
2. **Core API**
3. **Protocol**
4. **Tone/character encoding**
5. **Physical modem**
6. **USB serial terminal interface**

The PC-to-Arduino terminal interface uses **115200 8N1**, while Arduino-to-Arduino communication uses the dedicated **D9/D8 Timer1 tone modem**.
