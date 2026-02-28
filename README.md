# Indroduction







## Important: Don’t Confuse AI Depth with Physical Stereo

For NIR stereo tracking, **AI is not required**.

EdgeTrack is built on **deterministic, geometry-based NIR stereo vision** as the primary measurement system. Depth is computed from **physical baseline geometry, calibrated optics, and synchronized capture** — not from learned priors or dataset inference.

AI is **optional** and only used where it provides measurable benefit, such as:

* Stability monitoring
* Lightweight classification
* Left/right consistency checks
* Failure detection and recovery
* Semantic interpretation (gesture meaning, object type, intent)

The **core 3D reconstruction pipeline remains purely geometric and metric**.

### Why Multi-View Geometry Is Stronger Than AI Guessing

A single stereo rig can benefit from AI assistance in difficult cases.

However, the structurally stronger solution is **multi-view geometry**.

With **2–3 synchronized stereo rigs**, the system gains:

* Reduced occlusions
* Redundant triangulation
* Cross-validation between viewpoints
* Increased robustness without relying on learned priors

In many real-world setups, this reduces the need for AI almost entirely.

Geometry scales predictably.
Inference does not.

### Example: Apple’s Depth Pro

Apple’s Depth Pro is a powerful **monocular AI depth model**.

It is technically impressive — but it does not replace physical stereo measurement.

* AI depth infers structure from statistical patterns learned during training.
* Stereo depth measures disparity derived from real-world geometry and baseline separation.

Both approaches are valid — but they serve different purposes.

In EdgeTrack, AI models act as **assistive layers**, not as the measurement foundation.

### Physical Stereo vs Neural Stereo vs AI Depth

| Property                   | Classic Stereo (Geometry)       | Neural Stereo (AI-assisted matching) | Monocular AI Depth                |
| -------------------------- | ------------------------------- | ------------------------------------ | --------------------------------- |
| **Number of cameras**      | 2                               | 2                                    | 1                                 |
| **Requires baseline**      | Yes                             | Yes                                  | No                                |
| **Depth principle**        | Triangulation from disparity    | Learned disparity estimation         | Learned depth inference           |
| **Metric scale accuracy**  | True metric (after calibration) | True metric (after calibration)      | Often relative unless constrained |
| **Determinism**            | High (repeatable geometry)      | Medium (model-dependent)             | Low (probabilistic inference)     |
| **Training required**      | No                              | Yes                                  | Yes                               |
| **NPU/GPU requirement**    | No                              | Often beneficial                     | Required for real-time            |
| **Per-frame latency**      | Low, predictable                | Medium to high                       | Medium to high                    |
| **Multi-view consistency** | Naturally consistent            | Needs fusion logic                   | Needs full reconstruction logic   |
| **Texture-poor surfaces**  | Can struggle without pattern *  | Often improved                       | Often improved                    |
| **Gloss / reflections**    | Controlled with NIR + filtering | Model-dependent                      | Can hallucinate                   |
| **Occlusion handling**     | Solved via multi-view geometry  | Improved with fusion                 | Must infer hidden geometry        |
| **Interpretability**       | Physical measurement            | Hybrid                               | Statistical estimate              |

\* In controlled indoor setups (no direct sunlight, minimal glare, stable NIR illumination), stereo performs extremely well.
Failure modes typically appear on:

* Very low-texture materials
* Strong specular reflections
* Severe occlusions

EdgeTrack addresses these primarily through **optics and geometry**, not neural compensation.

The current prototype uses **diffuse (matte) NIR illumination** to generate a clean, homogeneous flood field. This improves stereo correspondence and reduces hotspots that destabilize matching.

On top of that, **MultiView coverage (2–3 rigs)** significantly increases robustness by:

* Reducing occlusions
* Increasing usable perspective diversity
* Improving triangulation reliability

Structured-light projection can improve extreme texture-poor scenes. However, it adds cost, optical complexity, synchronization constraints, and cross-talk risks — often without clear benefit for EdgeTrack’s primary use case.

For **close-range hand interaction (0.5–0.8 m)**, natural micro-texture of skin and clothing combined with controlled NIR lighting is typically sufficient. Structured light is therefore unnecessary in most scenarios.

### Architectural Positioning

EdgeTrack follows a strict hierarchy:

1. **Primary layer:** Deterministic NIR stereo geometry
2. **Secondary layer (optional):** AI-based refinement or validation
3. **Semantic layer:** Gesture interpretation and high-level understanding

This avoids a common market confusion:

> AI depth estimation is not equivalent to physical stereo measurement.

Stereo **measures**.
Neural stereo **optimizes matching**.
Monocular AI **estimates**.

When combined properly, they complement each other — but they are not interchangeable.

---

## Comparison of Available Hardware on the Market

> **Note:** This comparison focuses on architectural design principles and system integration models.
> Feature availability varies by firmware version, SDK access level, and configuration.
> Specifications are summarized at a high level and may differ by exact model variant.

---

| Feature / Focus                               |         ZED 2i         |   RealSense (e.g., D455)  |          Bumblebee X 5GigE          | Leap Motion (Controller 2) |          OptiTrack         |  Basler Stereo (stereo ace)  | Orbbec (Gemini 2) |              EdgeTrack             |
| --------------------------------------------- | :--------------------: | :-----------------------: | :---------------------------------: | :------------------------: | :------------------------: | :--------------------------: | :---------------: | :--------------------------------: |
| **Primary use case**                          |  Robotics / XR / Depth |       Depth sensing       |       Industrial stereo depth       |        Hand tracking       |     Marker-based MoCap     |    Industrial stereo depth   |   Depth sensing   | Editor authoring / pro interaction |
| **Typical interface**                         |           USB          |            USB            |           5GigE (Ethernet)          |             USB            |   Ethernet (system-based)  | GigE / USB (model dependent) |        USB        |        Ethernet (multi-rig)        |
| **Stereo depth**                              |           🟢           |             🟢            |                  🟢                 |             🔴             |             🔴             |              🟢              |         🟢        |                 🟢                 |
| **On-device depth compute**                   |  🔴 (host GPU typical) | 🟢 (dedicated depth ASIC) |   🟢 (on-board stereo processing)   |             🔴             |             🔴             |  🟢 (camera-based disparity) |  🟢 (custom ASIC) |     🟢 (ROI-based edge compute)    |
| **AI/VPU-style accelerator**                  |           🔴           |  🔴 (depth ASIC ≠ AI VPU) |                  🔴                 |             🔴             |             🔴             |              🔴              |         🔴        |    Optional (platform-dependent)   |
| **FPGA-based stereo pipeline**                |           🔴           |             🔴            |  🟢 (industrial hardware pipeline)  |             🔴             |             🔴             |              🔴              |         🔴        |                 🔴                 |
| **Open RAW sensor access**                    | 🟡 (limited SDK modes) | 🟡 (not typical workflow) | 🟢 (12-bit rectified stereo option) |             🔴             |             🟡             |              🟢              |       🔴/🟡       |                 🟢                 |
| **Native multi-device fusion**                |           🔴           |             🔴            |                  🔴                 |             🔴             | 🟢 (system-level software) |              🔴              |         🔴        |                 🟢                 |
| **Deterministic timing layer**                |           🔴           |             🔴            |     🟡 (industrial sync support)    |             🔴             |             🟢             |              🔴              |         🔴        |                 🟢                 |
| **Linux-based edge OS on device**             |           🔴           |             🔴            |                  🔴                 |             🔴             |             🔴             |              🔴              |         🔴        |                 🟢                 |
| **Open-source core**                          |           🔴           |             🔴            |                  🔴                 |             🔴             |             🔴             |              🔴              |         🔴        |                 🟢                 |
| **Typical depth range (manufacturer stated)** |       ~0.3–20 m*       |         ~0.4–6 m*         |              ~0.3–10 m*             |          ~0.1–1 m*         |         ~0.2–20 m*         |          ~0.2–10 m*          |    ~0.15–10 m*    |             0.1–10 m**             |

---

## Footnotes

* Manufacturer-stated operational ranges under ideal conditions.
Actual performance depends on lighting, surface texture, calibration, and environmental conditions.

** EdgeTrack is optimized for **high-precision operation in the near field (≤ ~1.2 m)** using homogeneous NIR flood illumination.
Extended ranges up to ~10 m are configuration-dependent and may require:

* VCSEL dot-pattern projection (active stereo assist)
* Increased baseline
* Higher-power NIR flood illumination
* Environment-dependent neural stereo refinement

Depth precision and usable range depend on baseline geometry, optics, illumination design, and processing strategy.
For transparency, geometric performance relationships are documented separately via disparity-based calculation tools.

---

## ⚙️ VPU vs CPU (Stereo Disparity)

As described above, many classic stereo/depth cameras come with limitations - but they also have a clear advantage: an on-device VPU/ASIC can compute disparity very efficiently.

The Raspberry Pi 5 has a natural trade-off: it is not as efficient at full-frame dense disparity processing. However, EdgeTrack follows a different strategy: direct RAW access enables a fully controllable pipeline where ROI selection, disparity range tuning, and targeted (sparse/ROI) matching can dramatically reduce compute cost while improving determinism and reproducibility.

Here is a quick comparison table:

| Feature                                 | 🖥️ CPU (Pi 5) | 🖥️ CPU (Threadripper) | 📟 VPU (On-device depth) |
|------------------------------------------|:-------------:|:----------------------:|:------------------------:|
| 🗺️ Dense depth efficiency               | ⚠️ Medium (CPU-limited) | 🚀 Very High (brute-force compute) | ✅ High (hardware-accelerated) |
| 🎥 720p @ 30 FPS (dense)                | ⚠️ Borderline (depends on disparity range) | ✅ Stable | ✅ Stable |
| ⚡ 120 FPS (dense)                      | ❌ Not practical | ⚠️ Possible (heavy power usage) | ❌ Not typical |
| 🎯 ROI matching (targeted processing)   | ✅ Very strong | ✅ Very strong | ⚠️ Limited |
| 🎞️ RAW control / pipeline freedom      | ✅ Full control | ✅ Full control | ⚠️ Limited |
| 🔗 Multi-rig sync / deterministic phase | ✅ Ideal | ⚠️ Complex (host-dependent) | ⚠️ Limited (device/framework dependent) ||

**In short:**
📟 VPU is great for **“easy dense depth output.”**
🖥️ CPU/RAW-first is great for **“controlled, repeatable geometry”** and **multi-rig workflows.**

---

## Why RAW Stereo Capture Instead of H.265

Most commercially available stereo and depth cameras rely on **H.264/H.265 video compression** to reduce bandwidth and simplify integration. While this approach is sufficient for visualization, XR previews, and general perception tasks, it introduces **lossy compression artifacts, temporal smoothing, and non-deterministic frame behavior** that negatively affect precise 3D reconstruction.

For high-accuracy stereo vision, **uncompressed RAW sensor data** is fundamentally superior. Using **RAW10** preserves the original linear pixel intensities produced by the image sensor, without spatial or temporal loss. This enables more robust stereo matching, accurate sub-pixel disparity estimation, and consistent depth reconstruction across frames.

In addition, RAW capture allows precise control over **exposure, gain, and synchronization**, which is essential for deterministic multi-camera systems. When combined with **near-infrared (NIR) illumination**, RAW stereo becomes significantly more stable under varying lighting conditions and supports reliable operation in controlled and industrial environments.

For these reasons, the project deliberately avoids H.265-based pipelines and focuses on **RAW10 stereo capture**, prioritizing determinism, precision, and authoring-grade 3D data quality over bandwidth efficiency.

> Note: H.265 is common for AI segmentation streams, but compression artifacts can bias models. In EdgeTrack, **geometry is the baseline**; AI is optional.

---

## Interface: USB vs. Ethernet vs. WLAN

USB, Ethernet, and WLAN are all commonly used to connect cameras and tracking devices, but they differ significantly in terms of determinism, scalability, and reliability.

**USB** is widely available and easy to set up, making it well suited for single-device configurations, prototyping, and consumer peripherals. However, USB is typically host-driven and shared across multiple devices on the same controller. As a result, bandwidth contention, variable latency, and timing jitter can occur, especially in multi-camera setups or under high system load. While acceptable for many use cases, these characteristics can limit predictability in time-critical pipelines.

**Ethernet** is designed for distributed and scalable systems. Each device operates independently on the network and commuates using explicit packetization, buffering, and timestamps. This enables more predictable latency, cleaner synchronization across multiple devices, and stable performance over longer cable distances. Ethernet also supports structured topologies using switches, VLANs, and Power-over-Ethernet (PoE), making it well suited for multi-rig and multi-room setups. For professional capture and deterministic processing pipelines, Ethernet is often the preferred transport layer.

**WLAN (Wi-Fi)** provides flexibility and mobility by removing physical cables, which can be advantageous in portable or rapidly reconfigurable environments. However, wireless links are inherently subject to interference, variable airtime, and changing network conditions. These factors can introduce fluctuating latency, packet loss, and jitter, which complicate synchronization and reproducibility. While modern Wi-Fi standards offer high peak bandwidth, sustained real-time performance is harder to guarantee.

In practice, USB is convenient for simple setups, Ethernet offers the most control and scalability for deterministic systems, and WLAN trades predictability for mobility and ease of deployment.

> **Note:** For tight timing, direct NIC connections are preferred. Switches usually add small latency, but can add variability under congestion; use QoS/VLAN/PTP if deterministic timing is required.

---

## Latency Comparison (Typical, No Aggressive Tuning)

| # | Class                                                         | Typical latency (no tuning)                                          | Jitter / determinism                            | Main latency contributors                                                                       | Practical notes                                                                                                                                                                                                                                     |
| - | ------------------------------------------------------------- | -------------------------------------------------------------------- | ----------------------------------------------- | ----------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1 | **High-end USB webcam (UVC, often ISP + MJPEG/H.264)**        | **50–200 ms** (often “noticeably laggy”)                             | **poor–medium**, highly variable                | Camera ISP + **multi-frame buffering** + (optional) codec/decode + host color/format conversion | Often “beautified” internally (AE/NR/scaling) and buffered for smooth video. Usually not suitable for low-latency VR/tracking workloads.                                                                                                            |
| 2 | **Industrial USB (USB3 Vision, often uncompressed/RAW/mono)** | **10–30 ms**                                                         | **medium**                                      | Sensor (exposure/readout) + USB transfer + host queue/copy                                      | Much better than webcams. **Bottlenecks:** USB bandwidth per host controller, cable/EMI constraints, and multi-camera setups saturate quickly. Multi-view can heavily load the host (DMA + CPU/GPU processing).                                     |
| 3 | **Industrial GigE (GigE Vision, UDP)**                        | **15–40 ms**                                                         | **medium**, jitter possible                     | Packetization + NIC/kernel/queueing + (optional) switch/network effects                         | Robust and flexible (long cables, PoE options). Without tuning you can see **jitter/spikes** (network contention, driver scheduling, interrupts). Multi-camera works, but host + network design becomes critical.                                   |
| 4 | **CoaXPress (frame grabber)**                                 | **5–20 ms**                                                          | **good–very good**                              | Sensor dominates; transport is almost “invisible”                                               | Highly deterministic with very fast triggering. **Downside:** **expensive** (cameras + frame grabber + cables) and a heavier industrial ecosystem. Multi-camera scales well, but cost/integration is high.                                          |
| 5 | **EdgeTrack (edge compute + results over Ethernet only)**     | **5–15 ms (keypoints/ROI)** / **10–30 ms (dense depth/point cloud)** | **very good**, deterministic (HW trigger + MCU) | Sensor + edge compute time (minimal transport overhead)                                         | You “spend” latency on **edge compute**, but save a lot elsewhere: no video codec, no RAW transport, and far less host load. Multi-view scales better because the host fuses **small result streams** (CoreFusion) instead of handling full frames. |

### Why #5 often “feels” best in real systems

* For **#1–#4**, the host typically has to **move frames + decode + convert formats + run vision**, which adds latency, jitter, and CPU/GPU load.
* For **#5**, the expensive work happens **before transport**, and you transmit only **keypoints/ROI/point clouds**, leading to lower, more predictable latency and a simpler host pipeline.

---
































## My Architecture and Design — Unconventional and Beyond the Market

### 1. Clear Separation of Capture and Processing

#### Problem

Native **MIPI CSI** camera interfaces offer excellent bandwidth, low latency, and precise timing, but they suffer from a major limitation: **very short cable lengths**, which makes larger or distributed camera setups impractical.

**USB-based** camera solutions allow longer cables, but USB is **interrupt-driven and host-dependent**, which typically results in **higher latency, increased jitter, and less deterministic timing**—especially problematic for tightly synchronized multi-camera systems.

**Ethernet/LAN-based** cameras improve distance and deployment flexibility, but many implementations still rely on **OS-level interrupts, buffering, and packet scheduling**, which are not inherently optimized for **hard real-time capture** or frame-accurate multi-camera synchronization. In practice, achieving truly deterministic behavior often requires a **real-time–tuned OS and network stack**, along with careful system-level optimization.

In addition, purpose-built **GigE / 2.5GigE machine-vision cameras** remain relatively expensive—often **€500+ per unit**—which makes large multi-camera arrays **cost-heavy** and limits scalability from a hardware-budget perspective.

At the high end, **CoaXPress** can deliver outstanding performance with direct, low-latency data paths into CPU/GPU memory. However, it comes with **very high hardware cost**, requires dedicated **frame grabbers**, and scales poorly **in terms of system cost and integration complexity**. Beyond the capture hardware itself, processing **multiple high-resolution cameras on a single host** can place a substantial load on the CPU/GPU—often pushing systems toward high-end workstation-class hardware (e.g., Threadripper-class systems) and significant engineering effort to optimize the processing pipeline.

#### Solution

EdgeTrack separates **image capture** from **high-level processing** by moving **reconstruction and preprocessing directly to the edge**. Instead of concentrating the entire workload on a single, expensive host system, each edge device performs its **local reconstruction tasks** using native MIPI CSI—where it performs best—and exports only **processed, compact 3D data** (e.g., keypoints, tool poses, sparse geometry) over the network.

This architecture preserves the **timing fidelity and signal quality** of native CSI capture while remaining **cost-efficient, scalable, and deployment-friendly**.

---

### 2. Ethernet-Native Alternative to CoaXPress

A **CoaXPress-based** camera infrastructure is a high-end solution in terms of **bandwidth and timing precision**, but it is **cost-intensive** and requires substantial **integration effort**, including dedicated **frame grabbers** and complex host-side pipelines.

Instead, comparable practical precision **for the target outputs** can be achieved through a combination of:

- **Well-defined multi-view geometry**
- **Calibrated stereo triangulation**
- **Edge-side preprocessing**
- **Early fusion in CoreFusion**

By transmitting **stable 3D primitives** (e.g., keypoints, tool poses, sparse geometry) rather than raw video streams, the system delivers **reproducible, low-jitter 3D signals** with significantly lower bandwidth requirements and reduced host-side complexity.

For the intended application, this architecture can **approach CoaXPress-class results** for **pose/keypoint accuracy and temporal stability**, while offering:

- **Simpler integration**
- **Lower hardware and maintenance costs**
- **Much better scalability**

Additional rigs can be added via **standard LAN connections**, rather than consuming limited frame-grabber channels and centralized capture resources.

---

### 3. TDM Phase-Offset Capture for Deterministic Timing

EdgeTrack uses **phase-offset global-shutter capture via Time-Division Multiplexing (TDM)**.
Instead of exposing all cameras simultaneously, multiple stereo rigs are triggered in **time-interleaved phases**.

This design:

* **Reduces occlusion**
* Improves **temporal consistency**
* Enables more **stable, repeatable input**

—especially important in **close-range, tool-centric workflows** where precision matters more than visual realism.

EdgeTrack is **markerless by default**, but supports **optional, minimal markers** when additional robustness is required.
Examples include subtle fingertip markers or markers placed directly on a tool. A “3D pencil” assisted by two small markers can provide **pen-like precision**, enabling reliable writing gestures or even **virtual keyboard interaction**.

A small **MCU-based trigger controller** generates deterministic, phase-shifted triggers for **up to eight stereo rigs** at **120 FPS per rig**.
When fused in **CoreFusion**, this results in an **effective aggregate update rate of up to ~960 Hz**, while maintaining **low jitter and high temporal stability**, depending on configuration and synchronization.


#### Why TDM Is Not Distributed Over LAN (and Why an MCU Can Still Make Sense)

A dedicated MCU (e.g., **RP2040**) is **not strictly required**—on a **Raspberry Pi 5**, TDM trigger signals can be generated locally using **hardware timers and/or DMA-driven GPIO** with sufficient precision for **120 FPS**.

The key distinction is **where timing is generated**:

* **Ethernet/LAN is excellent for data transport** (payload streaming, timestamps, configuration/control messages).
* But it is **not ideal as a real-time trigger bus**, because packet delivery depends on **OS scheduling, buffering, interrupts/NAPI, NIC behavior, and switch latency**, which introduces **variable jitter**.

Therefore, EdgeTrack uses **LAN for payload and timestamps**, while **TDM phase triggering is generated locally on each edge device** (Pi-side or MCU-side). If multiple edge devices must share a common phase reference, synchronization is handled via a **deterministic wired sync bus** (e.g., **RS-485**) or a **shared time base** (e.g., clock sync + scheduled start times), rather than sending **per-frame triggers** over the network.

> In short: **LAN transports data; the edge generates timing.**

### 4. Short Features

#### Stereo-First, Not AI-First

EdgeTrack uses **NIR stereo vision** as the primary tracking method. Depth is computed through **triangulation**, which means the system measures real geometry instead of guessing it. This makes the output predictable, repeatable, and easier to validate—especially important in professional workflows where stability matters more than “impressive demos.”

#### Designed for Short-Exposure Motion Freeze

The front-facing NIR illumination is not decorative. It enables **ultra-short exposure times** that freeze fast motion and reduce blur. This improves stereo correspondence, reduces noise, and stabilizes tracking under real-world movement—something many consumer-grade depth solutions struggle with.

#### MultiView as the Main Upgrade Path

Instead of relying on heavier models to “fix” failures, EdgeTrack scales through **MultiView geometry**. With **2–3 stereo rigs**, occlusions are reduced and robustness increases dramatically. In many scenarios, MultiView delivers reliability that would otherwise require complex AI, but without losing determinism.

#### Edge Processing, Minimal Data Output

EdgeTrack is designed to process data on the edge and export only what matters:

* 3D keypoints
* ROI point clouds
* compact geometric results

This reduces bandwidth, lowers latency, and keeps the system scalable for multi-sensor setups.

#### Optional AI as a Support Layer

AI can be added where it truly helps—such as plausibility checks, lightweight classification, or recovery in difficult edge cases. But AI is not the core. The core is a system you can measure, tune, and trust.

---



































## LiDAR and ToF

LiDAR and Time-of-Flight (ToF) sensors are frequently used for 3D perception, but they introduce trade-offs that can be problematic for **high-precision, close-range authoring and interaction** workflows.

**Key limitations in this context:**

* **Effective spatial resolution at close range:** Many ToF/LiDAR modules—especially compact or consumer-class devices—provide lower effective spatial detail than multi-view stereo, which limits fine hand, finger, or tool-level tracking.
* **Unstructured data for interaction:** Dense depth/point outputs are not automatically useful for control. Interaction benefits most from **stable structure** (keypoints, edges, marker geometry, ROI constraints) rather than uniform depth everywhere.
* **Temporal noise and instability:** Depth measurements can be affected by multi-path interference, surface reflectivity, ambient IR, and sensor-internal filtering, causing depth flutter and jitter—undesirable for repeatable input.
* **Multi-view scaling complexity:** Scaling active depth sensors across multiple viewpoints can increase cost and complexity due to emitter interference management (time-multiplexing, frequency/coding separation) and synchronization constraints.
* **Limited deterministic control:** Many modules behave as closed systems with fixed timing and internal depth processing, offering limited external synchronization and reduced transparency for frame-accurate multi-sensor fusion.
* **Integration and cost overhead:** High-quality LiDAR/industrial ToF systems with low noise and good stability are often expensive and may require proprietary SDKs and constrained deployment workflows.
* **Near-field suitability:** Many LiDAR/ToF systems are optimized for mid-to-long range. For desktop-scale workspaces, **stereo vision with controlled illumination** often provides higher effective precision and better repeatability.

**Short:** LiDAR/ToF sensors typically increase BOM, calibration, and integration complexity. In contrast, a simple global-shutter mono camera is easier to manufacture and scale.

**EdgeTrack design choice:** EdgeTrack deliberately favors synchronized global-shutter stereo vision with controlled NIR illumination, enabling:

* higher spatial detail at close range
* deterministic timing via external strobe and phase control
* better scalability across multiple rigs
* lower system and integration cost
* full control over the capture and reconstruction pipeline

For deterministic 3D authoring, precise hand/tool interaction, and reproducible editor workflows, multi-view stereo with explicit timing control is often a more accurate, scalable, and transparent foundation than LiDAR/ToF-based systems.

---

















## ⭐ Pixel format – preference

| Format                     |   Rating   | Comment                                                                                  |
| -------------------------- | ---------- | ---------------------------------------------------------------------------------------- |
| **MJPEG / JPEG**           | ⭐☆☆☆☆     | Only for preview/debug. Strong artifacts, variable bitrate, poor for precise 3D.         |
| **YUV / YUYV / NV12**      | ⭐⭐☆☆☆    | OK if you only use the **Y (luma)** channel. Extra bandwidth wasted on color info.       |
| **RAW8 / Y8 (8-bit mono)** | ⭐⭐⭐☆☆   | Solid baseline. Lower dynamic range, but good enough with proper NIR illumination.       |
| **RAW10**                  | ⭐⭐⭐⭐☆  | Very good: higher dynamic range, finer quantization, still manageable bandwidth.         |
| **RAW12**                  | ⭐⭐⭐⭐⭐ | Ideal for high precision: maximum dynamic range and depth resolution, highest bandwidth. |

---

## ⚙️ Quick Engineering Comparison — What is the best interface for deterministic vision?

When designing a machine-vision or stereo system, the choice of sensor interface has a strong impact on latency, control, and system complexity.
Below is a simplified engineering comparison:

| Interface              | Additional Chips / Infra | RAW Access | Latency     | Determinism |
| ---------------------- | ------------------------ | ---------- | ----------- | ---------- |
| **MIPI CSI-2**         | very few                 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **USB2 (typical UVC)** | medium                   | ⭐☆☆☆☆     | ⭐⭐☆☆☆     | ⭐☆☆☆☆     |
| **USB3 (typical UVC)** | medium                   | ⭐⭐☆☆☆    | ⭐⭐⭐☆☆   | ⭐⭐☆☆☆     |
| **GigE / GigE Vision** | many                     | ⭐⭐⭐⭐☆  | ⭐⭐⭐☆☆   | ⭐⭐⭐⭐☆  |
| **CoaXPress**          | heavy (framegrabber)     | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

### Summary

**MIPI CSI-2** is typically the best choice when deterministic timing, minimal latency, and direct RAW sensor access are required. The sensor is connected almost directly to the SoC, which reduces hidden processing stages and keeps the pipeline transparent.

**USB cameras** usually include additional ISP and bridge chips. They are convenient and plug-and-play, but often introduce internal processing and buffering that reduce determinism.

**GigE cameras** are powerful for industrial networking and long cable distances, but typically require more intermediate logic (FPGA/ASIC, packetization, buffering), which increases system complexity.

**CoaXPress** is a high-end industrial interface designed for very high bandwidth and deterministic transmission. It typically requires a dedicated frame grabber card on the host side and specialized hardware inside the camera. While it offers excellent throughput, low latency, and strong determinism, it significantly increases system cost, hardware complexity, and power requirements compared to embedded MIPI-based designs.

For edge-processing architectures focused on precise timing and reproducible results, **MIPI CSI-2 provides the most transparent and controllable capture path**.










