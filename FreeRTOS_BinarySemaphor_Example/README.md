# STM32F411 FreeRTOS Basic Demo  
## (LED Task + Button Interrupt with Binary Semaphore)

---

## 🧩 Project Overview
This project demonstrates **basic FreeRTOS multitasking and interrupt synchronization** on the STM32F411 microcontroller.

It includes:
- A **periodic LED task** (heartbeat)
- A **button interrupt** that triggers a task via a **binary semaphore**

These two examples form the foundation for understanding **task scheduling** and **interrupt-based synchronization** in FreeRTOS.

---

## ⚙️ Hardware Setup

| Component | Description | Pin |
|------------|--------------|-----|
| **MCU** | STM32F411CEU6 (Black Pill) | – |
| **LED1** | Heartbeat indicator | PA1 |
| **LED2** | Button event indicator | PA2 |
| **Button** | External user button (pull-down) | PC15 |

---

## 🧠 System Architecture

```mermaid
flowchart TD

subgraph FreeRTOS_Scheduler["FreeRTOS Scheduler"]
    direction TB
    T1["TaskLed\n(Heartbeat - 500ms Toggle)"]
    T2["TaskButton\n(Wakes on Binary Semaphore)"]
end

Button["User Button (PC15)\nInterrupt Source"]
ISR["EXTI Interrupt Service Routine\n(HAL_GPIO_EXTI_Callback)"]

Button -->|"Pressed"| ISR
ISR -->|"xSemaphoreGiveFromISR()"| Semaphore["Binary Semaphore"]
Semaphore -->|"xSemaphoreTake() waits"| T2

T2 -->|"LED2 ON 500ms + UART log"| LED2["PA2 LED"]
T1 -->|"500ms Toggle"| LED1["PA1 LED"]
