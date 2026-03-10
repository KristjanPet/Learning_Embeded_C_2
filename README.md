# learning_embeded_c_2 — ESP32 Learning Topics (Scratch Repo)

This repository is a **topic-by-topic learning playground** for ESP32 using **ESP-IDF + PlatformIO (C/C++)**.

## Purpose

This repo is **not** meant to be a clean, production-style codebase.

It exists so I can:
- learn **separate embedded topics** in small, practical exercises,
- keep each topic isolated enough to review later,
- use real tools (logic analyzer, debugger, etc.) while iterating quickly.

> If you are looking for a clean, structured project: that will be in a **different repository / different chat**.

## Important Notes (Read This First)

- This repo **does not follow a strict architecture** and may contain experiments, quick hacks, and throwaway code.
- Each learning topic is implemented and reviewed as a **separate Pull Request**.
- The “final” state of `main` is just the accumulation of merged topic PRs and might include leftovers from experiments.
- To study a specific topic, check the **Merged Pull Requests** and review the commit/PR history.

## How to Navigate Topics

**Each topic lives in its own merged PR.**

Recommended workflow:
1. Go to your Git hosting (GitHub/GitLab).
2. Open **Pull Requests** → filter to **Merged**.
3. Pick a topic PR and review:
   - description
   - files changed
   - commits
   - discussion notes (if any)

## Toolchain

- Board: **ESP32 DevKitC (WROOM)**
- Framework: **ESP-IDF**
- Build system: **PlatformIO**
- Language: **C/C++** (prefer C++ style where practical)

## Build / Flash / Monitor (PlatformIO)

```bash
pio run -t upload
pio device monitor
```

## Topics Covered (high level)

Examples of topics that may appear as merged PRs:
- Hardware timers (gptimer) + measurement with logic analyzer
- DMA concepts (practical SPI example)
-  Clock / timing pitfalls (practical)
-  Memory management (stack vs heap, high-water marks)
-  Deep sleep + wake-up sources
-  Boot/partition/OTA concepts (overview)
-  ESP-IDF components + CMake dependency rules (REQUIRES vs PRIV_REQUIRES)
-  RS485 (MAX485) + framing/CRC/timeouts + ACK/retry
-  Debugging workflows using tools (logic analyzer, serial, etc.)

The definitive list is the Merged PR history.
