# ZC2X Copilot Instructions

You are contributing to the ZC2X project.

## Mission

ZC2X is an open-source embedded telemetry framework written in C using ESP-IDF.

The immediate goal is to deliver a reliable MVP for Formula SAE telemetry.

Current data path:

CAN
→ OBU
→ XBee
→ RSU
→ NATS Core

Always optimize for simplicity, reliability and incremental development.

---

# Architecture

Before implementing any feature, read the RFCs under:

docs/architecture/

The RFCs are the source of truth.

Do not invent new architecture.

Do not redesign existing architecture.

---

# Current Priority

Implement only what is necessary for the MVP.

Priority order:

1. CAN reception
2. Packet creation
3. XBee transmission
4. RSU reception
5. NATS Core publishing

Everything else comes later.

---

# Coding Style

- Language: C99
- Platform: ESP-IDF
- Keep functions small.
- Keep modules independent.
- Avoid unnecessary abstractions.
- Avoid dynamic allocation unless required.
- Prefer explicit code over clever code.

---

# Repository Organization

framework/
Platform-independent code.

platform/esp-idf/
ESP-IDF specific implementation.

applications/
Firmware entry points.

Never move platform-specific code into framework.

---

# Development Rules

Always explain the implementation plan before generating code.

Generate complete files.

Never generate placeholder code.

Never generate TODO-only modules.

Every commit should compile.

Every module should have a single responsibility.

---

# Packet Philosophy

Everything becomes a packet.

Packets are immutable after creation.

Transports never modify packet contents.

Storage stores packets exactly as produced.

---

# CAN Philosophy

The OBU never decodes CAN.

The OBU forwards raw CAN frames.

DBC decoding happens only after the data reaches HQ.

---

# When uncertain

Prefer the simplest implementation that satisfies the RFC.
