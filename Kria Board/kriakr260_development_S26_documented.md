# KR260 Complete Beginner Guide
## From Unboxing to Running a Custom HLS IP Block on Real Hardware

**Who this is for:** So you have unboxed a Kria KR260 board
and want to understand exactly what to do, what every button means, and why each step exists.
No prior FPGA or Vivado experience assumed.

**What you will build:** A hardware pipeline where Python sends an array of numbers to the FPGA,
the FPGA doubles every value using a circuit you designed in C++, and Python reads the result back.

**Tools used:**
- **Vivado 2025.2** — AMD's FPGA design environment (runs on your development PC)
- **Vitis HLS 2025.2** — The tool that compiles C++ into hardware (runs on your development PC)
- **PYNQ** — A Python framework that lets you control the FPGA from Python (runs on the Kria board)

---

## Table of Contents

1. [Concepts You Need to Know First](#1-concepts-you-need-to-know-first)
2. [Hardware Setup — Kria Board First Boot](#2-hardware-setup--kria-board-first-boot)
3. [Install PYNQ on the Kria](#3-install-pynq-on-the-kria)
4. [Install Vivado on Your Development PC](#4-install-vivado-on-your-development-pc)
5. [Create a Vivado Project](#5-create-a-vivado-project)
6. [Build the Block Diagram](#6-build-the-block-diagram)
7. [Generate the Bitstream](#7-generate-the-bitstream)
8. [Write the Scalar Multiplier in Vitis HLS](#8-write-the-scalar-multiplier-in-vitis-hls)
9. [Add the HLS IP to Vivado](#9-add-the-hls-ip-to-vivado)
10. [Deploy to the Kria Board](#10-deploy-to-the-kria-board)
11. [Run the Python Test](#11-run-the-python-test)
12. [Troubleshooting Reference](#12-troubleshooting-reference)

---

## 1. Concepts You Need to Know First

Before touching any software, read this section. These concepts will make every subsequent
step make sense instead of feeling like magic incantations.

### What is an FPGA?

A regular CPU executes instructions one after another, like a chef following a recipe step
by step. An **FPGA (Field-Programmable Gate Array)** is different — you are literally
configuring the *wiring* of a chip. Instead of a program that runs on fixed hardware, you
describe a circuit, and the FPGA becomes that circuit.

The advantage: your circuit runs in parallel, continuously, at clock speed. A 200 MHz FPGA
can process one piece of data every 5 nanoseconds with zero software overhead.

The tradeoff: you have to design the circuit, which requires different tools and a different
way of thinking compared to software.

### What is the Zynq UltraScale+ MPSoC?

The K26 SOM (System on Module) at the heart of the KR260 contains a special chip called
the **Zynq UltraScale+ MPSoC**. This chip combines two things on a single piece of silicon:

- **PS (Processing System):** A quad-core ARM Cortex-A53 processor. This is essentially
  a small computer. It runs Linux, Python, and everything you would normally run on a
  Raspberry Pi. When you SSH into the Kria or run a Python script, you are using the PS.

- **PL (Programmable Logic):** The FPGA fabric. This is the part you program with Vivado.
  It sits right next to the CPU on the same chip, connected by very fast internal buses.

Think of it like having a Raspberry Pi glued directly to an FPGA, with a superhighway
connecting them. The PS runs your software; the PL runs your custom hardware.

### What is a Bitstream?

When you design a circuit in Vivado and click "Generate Bitstream," Vivado compiles your
design into a binary file called a **bitstream** (`.bit` file). This file contains the
configuration data that tells every programmable switch inside the FPGA fabric what to
connect to what.

Loading a bitstream onto the Kria is like flashing firmware — it programs the hardware.
Unlike ROM, the FPGA forgets its configuration when power is removed, so you reload the
bitstream every time the board boots.

### What is an IP Block?

**IP (Intellectual Property) blocks** are pre-built, reusable hardware modules. In Vivado,
you build designs by connecting IP blocks together in a visual diagram — similar to how you
might connect components on a schematic. AMD provides hundreds of IP blocks (DMA controllers,
FFTs, GPIOs, memory interfaces, etc.) and you can also create your own using Vitis HLS.

### What is AXI?

**AXI (Advanced eXtensible Interface)** is the standard communication protocol used between
IP blocks on AMD/Xilinx SoCs. Think of it like USB — it is a standardized way for hardware
modules to talk to each other. There are two main variants:

- **AXI4-Lite:** For slow register reads/writes (like configuring an IP block)
- **AXI4-Stream:** For fast streaming data (like sending an array of samples to the FPGA)

The PS uses AXI to talk to the DMA controller. The DMA controller uses AXI-Stream to talk
to your custom IP. This chain is how Python data ends up flowing through your circuit.

### What is DMA?

**DMA (Direct Memory Access)** is a mechanism that lets hardware read from and write to
RAM without involving the CPU for every byte. Without DMA, the CPU would have to manually
ferry each data sample from memory to your IP block — extremely slow. With DMA, you tell
it "transfer these 1024 numbers from address X to this hardware block," and it does so
automatically while the CPU can do other things.

In our design, Python allocates a buffer in RAM, fills it with data, and the DMA engine
moves it through the FPGA fabric and back.

### What is Vitis HLS?

**Vitis HLS (High-Level Synthesis)** is a compiler, but instead of producing machine code
for a CPU, it produces a circuit description (RTL — Register Transfer Level) for an FPGA.

You write a C++ function that describes the computation you want. Vitis HLS figures out
how to build a circuit that implements that computation in hardware. The output is an IP
block you can drop into your Vivado diagram.

This is the core workflow we use: write math in C++, synthesize it into hardware, connect
it to the DMA pipeline.

### What is PYNQ?

**PYNQ (Python Productivity for Zynq)** is a framework that makes it easy to interact
with FPGA hardware from Python. Normally, to load a bitstream on Linux and talk to the
hardware, you would need to write device tree files, kernel drivers, and memory-mapped
I/O code. PYNQ automates all of that.

With PYNQ, loading a bitstream is one line of Python:
```python
ol = Overlay("my_design.bit")
```
And sending data to the FPGA is another few lines using NumPy arrays. This is why we use
it for prototyping.

---

## 2. Hardware Setup — Kria Board First Boot

### What's in the Box

Your KR260 kit contains:
- The KR260 board itself
- A 12V power supply
- A USB-C cable (for the serial console — you probably won't need this)
- A microSD card (may or may not be pre-flashed)

You will also need:
- A microSD card (16GB minimum, 32GB+ recommended) — get one if not included
- An Ethernet cable to connect the board to your router
- A computer on the same local network

### Step 2.1 — Flash Ubuntu to the SD Card

The Kria boots its operating system from a microSD card, just like a Raspberry Pi.

**On your development PC:**

1. Download the **Ubuntu 22.04 LTS image for the K26 SOM** from AMD's website:
   - Go to: https://ubuntu.com/download/amd
   - Look for "Ubuntu Server 22.04 LTS for Kria KR260" and download the `.img.xz` file

2. Download and install **Balena Etcher** (free, works on Windows/Mac/Linux):
   - https://etcher.balena.io/

3. Insert your microSD card into your PC (use a USB adapter if needed)

4. Open Balena Etcher:
   - Click **"Flash from file"** → select the `.img.xz` you downloaded (Etcher can
     decompress it automatically)
   - Click **"Select target"** → choose your microSD card
     > ⚠️ Be careful here. Select the SD card, not your PC's internal drive.
   - Click **"Flash!"** and wait ~5-10 minutes

5. When flashing is complete, safely eject the SD card

### Step 2.2 — First Boot

1. Insert the flashed microSD card into the KR260's SD card slot (on the underside of
   the board, near the edge)

2. Connect an Ethernet cable from the KR260 to your router or network switch

3. Plug in the 12V power supply. The board will begin booting automatically.

4. Wait about 60-90 seconds for the first boot to complete

### Step 2.3 — Find the Board's IP Address

You need the Kria's IP address to connect to it. Two options:

**Option A (easiest):** Log into your home router's admin page (usually at 192.168.1.1
or 192.168.0.1) and look at the connected devices list. Find the device named "kria" or
"ubuntu."

**Option B:** If you have a monitor and keyboard, connect them to the board. Log in with:
- Username: `ubuntu`
- Password: `ubuntu`
Then run: `hostname -I`

### Step 2.4 — SSH into the Board

From a terminal on your PC:
```bash
ssh ubuntu@<kria_ip_address>
# Example: ssh ubuntu@192.168.1.45
```

When prompted, the password is `ubuntu`. You will also be asked to set a new password
on first login — set something you will remember.

You are now inside the Kria's Linux terminal. Everything from here until Step 3 happens
here.

### Step 2.5 — Update the System

```bash
# Update the package list and install upgrades
sudo apt update && sudo apt upgrade -y
```

This takes a few minutes. Press Enter if any prompts appear.

### Step 2.6 — Update the Boot Firmware

This is important. The KR260 has firmware in flash memory (separate from the SD card)
that must match the software version on the SD card. Mismatches cause mysterious failures.

```bash
sudo apt install -y xrt-dkms
sudo xmutil bootfw_update -i /lib/firmware/xilinx/kr260-startup/kr260-startup.bin
```

After this command completes, power cycle the board (unplug power, wait 5 seconds,
plug back in). SSH back in after ~90 seconds.

### Step 2.7 — Disable Automatic Updates

Ubuntu runs background updates that hold a lock file and will interfere with our software
installations. Disable them now:

```bash
sudo systemctl stop unattended-upgrades
sudo systemctl disable unattended-upgrades
```

---

## 3. Install PYNQ on the Kria

All of these commands run on the Kria over SSH.

### Step 3.1 — Clone the PYNQ Installer

```bash
git clone https://github.com/Xilinx/Kria-PYNQ.git
cd Kria-PYNQ/
```

### Step 3.2 — Run the Installer

```bash
sudo bash install.sh -b KR260
```

This takes approximately **25 minutes**. It installs JupyterLab, PYNQ, OpenCV, and all
dependencies. Do not close the SSH session while this runs.

If it fails with a lock error (`dpkg lock`), run:
```bash
sudo systemctl stop unattended-upgrades
sudo bash install.sh -b KR260
```
and it will pick up where it left off.

### Step 3.3 — Fix the NumPy Version Conflict

The PYNQ installer upgrades NumPy to version 2.x, but PYNQ's internal modules were
compiled against NumPy 1.x. This causes a crash on import. Fix it:

```bash
pip install "numpy<2" --force-reinstall
```

### Step 3.4 — Verify JupyterLab is Running

```bash
systemctl status jupyterlab
```

You should see `active (running)`. If not:
```bash
sudo systemctl start jupyterlab
sudo systemctl enable jupyterlab
```

### Step 3.5 — Test JupyterLab in Your Browser

On your PC, open a web browser and go to:
```
http://<kria_ip_address>:9090/lab
```
Password: `xilinx`

You should see the JupyterLab interface. This is where you will run Python code to
control the FPGA. Leave this tab open for later.

---

## 4. Install Vivado on Your Development PC

Vivado runs on your **development PC (Linux)**, not the Kria. The Kria is too slow to run
design tools — it just runs the final result.

### Step 4.1 — Download Vivado

1. Go to: https://www.xilinx.com/support/download.html
2. Find **Vivado ML Edition 2025.2** and download the Linux self-extracting installer
   (the `.bin.tar.gz` or web installer)
   > You will need to create a free AMD account

3. Make the installer executable and run it:
```bash
chmod +x Xilinx_Unified_2025.2_XXXX_Linux.bin
./Xilinx_Unified_2025.2_XXXX_Linux.bin
```

### Step 4.2 — Install Vivado

Follow the GUI installer:
- **Select product:** Vivado
- **Select edition:** Vivado ML Standard (free) or Enterprise if your lab has a license
- **Devices:** Make sure **Zynq UltraScale+ MPSoC** is checked. You can uncheck others to
  save disk space.
- **Install location:** `/tools/Xilinx/` (the standard location)
- Complete the installation (~30-60 minutes, ~70GB disk space)

### Step 4.3 — Source the Settings Script

Every time you open a new terminal and want to use Vivado, you must source this script:
```bash
source /tools/Xilinx/Vivado/2025.2/settings64.sh
```

To avoid doing this manually every time, add it to your `.bashrc`:
```bash
echo 'source /tools/Xilinx/Vivado/2025.2/settings64.sh' >> ~/.bashrc
source ~/.bashrc
```

### Step 4.4 — Launch Vivado

```bash
vivado &
```

The `&` runs it in the background so your terminal stays usable.

---

## 5. Create a Vivado Project

Everything in this section happens in the Vivado GUI on your development PC.

### Step 5.1 — Start the New Project Wizard

When Vivado opens, you will see the **Quick Start** panel. Click **"Create Project"**.

A wizard opens. Click **Next** on the first page (it just says "New Project").

### Step 5.2 — Project Name and Location

- **Project name:** Type `kria_dma_pipeline`
- **Project location:** Click the `...` browse button. Navigate to `/home/cpsl/` and
  create a new folder. Select it.
- Leave **"Create project subdirectory"** checked

Click **Next**.

### Step 5.3 — Project Type

Select **"RTL Project"**. This is the standard type for custom hardware designs.

Leave **"Do not specify sources at this time"** checked — we will add sources later.

Click **Next**.

### Step 5.4 — Select the Target Part

This is the most important step in the wizard. You are telling Vivado exactly which
FPGA chip you are targeting. If you get this wrong, the generated bitstream won't
work on your board.

1. Click the **"Boards"** tab at the top of the dialog
   > If you see "Parts" selected, click "Boards" instead. Boards includes the full
   > KR260 board definition which pre-configures some settings for you.

2. In the search box, type `KR260`

3. If you see **"Kria KR260 Robotics Starter Kit"** in the list, select it and click Next.

4. If the board does not appear, click the **"Refresh"** button, or switch to the
   **"Parts"** tab and search for: `xck26-sfvc784-2LV-c`
   - Family: Zynq UltraScale+
   - Package: sfvc784
   - Speed: -2LV

Click **Next**.

### Step 5.5 — Summary

Review the summary. You should see:
- Project Type: RTL Project
- Part: xck26-sfvc784-2LV-c (or the KR260 board)

Click **Finish**. Vivado will open your project.

---

## 6. Build the Block Diagram

The **block diagram** is where you visually connect IP blocks together to define your
hardware design. Think of it like drawing a circuit schematic, but with high-level
modules instead of individual transistors.

### What We Are Building

Our design has this data flow:

```
Python (RAM) → DMA → scalar_mult IP → DMA → Python (RAM)
```

The DMA reads data from Python's buffer in RAM, sends it through our multiplier circuit,
and the output goes back to RAM where Python can read it. The PS configures and controls
the DMA over a separate AXI control bus.

The supporting infrastructure we need:
- **Zynq PS** — the ARM processor (always required)
- **Clocking Wizard** — generates the PL clock signal (200 MHz)
- **Processor System Reset** — synchronizes resets properly
- **AXI SmartConnect** — routes AXI control traffic between PS and IP blocks
- **AXI DMA** — the direct memory access engine
- **SmartConnect (data)** — routes DMA data traffic to/from PS memory

### Step 6.1 — Open the Block Diagram Editor

In the Vivado left panel (Sources pane), you should see no sources yet. Look for the
**"IP INTEGRATOR"** section in the **Flow Navigator** on the very left side of the window.

Click **"Create Block Design"**.

A dialog appears:
- **Design name:** Leave as `design_1` or rename to `kria_bd`
- Click **OK**

A blank canvas opens in the main area. This is your block diagram editor.

### Step 6.2 — Add the Zynq UltraScale+ PS

The Zynq PS block represents the ARM processor. It is always the starting point.

**How to add an IP block (you'll do this many times):**
Right-click anywhere on the blank canvas → click **"Add IP"**
A search box appears. Type the name of the IP you want → double-click it to place it.

1. Right-click canvas → Add IP → type `zynq` → double-click **"Zynq UltraScale+ MPSoC"**

A large block appears on the canvas labeled "zynq_ultra_ps_e_0." This is the PS.

**Run Block Automation:**
You should see a green banner at the top of the canvas saying:
> "Designer Assistance available. Run Block Automation"

Click **"Run Block Automation"**. A dialog appears.

- Make sure the `zynq_ultra_ps_e_0` checkbox is checked
- Leave all settings at their defaults
- Click **OK**

> **What did this just do?** Block Automation applied a board preset — it configured
> the Zynq PS with the correct settings for the KR260 board (memory type, clock sources,
> peripheral assignments). Without this, you'd have to manually configure ~50 settings.

After Block Automation, you will see several additional ports appear on the Zynq block.

### Step 6.3 — Enable the HP0 Slave Port on the Zynq

The DMA will write data to RAM through a special high-performance port on the Zynq called
**HP0 (High Performance port 0)**. We need to enable it.

**Double-click the Zynq block** to open its configuration dialog. This is a large dialog
with many tabs on the left.

1. Click **"PS-PL Configuration"** in the left list
2. Expand **"PS-PL Interfaces"** → expand **"Slave Interface"** → expand **"AXI HP Slave FPD"**
3. Find **"AXI HPC0 FPD"** — make sure it is **checked/enabled**
   > HP = High Performance. FPD = Full Power Domain. This is the memory bus the DMA uses
   > to write results back to RAM.
4. Click **OK** to close the dialog

### Step 6.4 — Add the Clocking Wizard

The PS outputs a reference clock, but we want to generate our own 200 MHz clock for the
PL logic. The Clocking Wizard IP does this.

1. Right-click canvas → Add IP → type `clocking wizard` → double-click **"Clocking Wizard"**

2. **Double-click** the Clocking Wizard block to configure it:
   - Click the **"Output Clocks"** tab
   - **clk_out1:** Set to `200.000` MHz
   - Leave clk_out2 and others disabled (or they can remain, we only use clk_out1)
   - Click **OK**

3. Connect the clock input:
   - On the Zynq block, find the port **`pl_clk0`** (on the right side of the block)
   - On the Clocking Wizard, find the port **`clk_in1`** (on the left side)
   - **Click on `pl_clk0`** and drag to **`clk_in1`** to draw a wire
   
   > **How wiring works in Vivado:** Click on a port, hold the mouse button, drag to
   > the destination port, release. A green wire appears. If Vivado shows a green check
   > mark, the connection is valid. If it shows an X, the ports are incompatible.

4. Connect the reset input:
   - On the Zynq block, find **`pl_resetn0`**
   - On the Clocking Wizard, find **`resetn`**
   - Connect them with a wire

### Step 6.5 — Add the Processor System Reset

This IP synchronizes reset signals across clock domains so everything resets cleanly.

1. Right-click canvas → Add IP → type `processor system reset` → double-click
   **"Processor System Reset"**

2. Connect the clocks and resets:
   - `clk_wiz_0 clk_out1` → `proc_sys_reset_0 slowest_sync_clk`
   - `zynq_ultra_ps_e_0 pl_resetn0` → `proc_sys_reset_0 ext_reset_in`
   - `clk_wiz_0 locked` → `proc_sys_reset_0 dcm_locked`

### Step 6.6 — Add the AXI SmartConnect (Control Bus)

The PS needs a way to send control commands to the DMA (to tell it to start transfers,
configure addresses, etc.). The **AXI SmartConnect** routes these AXI control signals.

1. Right-click canvas → Add IP → type `smartconnect` → double-click **"AXI SmartConnect"**

2. **Double-click** the SmartConnect block to configure it:
   - **Number of Slave Interfaces:** `1` (only the PS connects as a master to it)
   - **Number of Master Interfaces:** `1` (it connects to the DMA's lite control port)
   - Click **OK**

3. Connect:
   - `zynq_ultra_ps_e_0 M_AXI_HPM0_LPD` → `axi_smc S00_AXI`
   - `clk_wiz_0 clk_out1` → `axi_smc aclk`
   - `proc_sys_reset_0 interconnect_aresetn[0:0]` → `axi_smc aresetn`

### Step 6.7 — Add the AXI DMA

The DMA is the heart of our data pipeline. It has two main data channels:

- **MM2S (Memory-Map to Stream):** Reads data from RAM and sends it out as a stream
  to your IP block. "Send channel" from Python's perspective.
- **S2MM (Stream to Memory-Map):** Receives a data stream from your IP block and writes
  it to RAM. "Receive channel" from Python's perspective.

1. Right-click canvas → Add IP → type `axi dma` → double-click **"AXI Direct Memory Access"**

2. **Double-click** the DMA block to configure it:
   - Uncheck **"Enable Scatter Gather Engine"** — we want simple mode, not scatter-gather
   - **Width of Buffer Length Register:** set to `23`
     > This controls the maximum DMA transfer size. 23 bits = 8MB max, which is plenty.
   - Leave MM2S and S2MM both enabled
   - Click **OK**

3. Connect the control port:
   - `axi_smc M00_AXI` → `axi_dma_0 S_AXI_LITE`
   - `clk_wiz_0 clk_out1` → `axi_dma_0 s_axi_lite_aclk`
   - `clk_wiz_0 clk_out1` → `axi_dma_0 m_axi_mm2s_aclk`
   - `clk_wiz_0 clk_out1` → `axi_dma_0 m_axi_s2mm_aclk`
   - `proc_sys_reset_0 peripheral_aresetn[0:0]` → `axi_dma_0 axi_resetn`

### Step 6.8 — Add a Second SmartConnect (Data Bus)

The DMA's data ports (MM2S and S2MM) need their own path to the Zynq's HP0 memory bus.
This is separate from the control SmartConnect.

1. Right-click canvas → Add IP → type `smartconnect` → double-click **"AXI SmartConnect"**
   (you will now have two — `axi_smc` for control and `smartconnect_0` for data)

2. **Double-click** `smartconnect_0`:
   - **Number of Slave Interfaces:** `2` (DMA MM2S and S2MM both connect here)
   - **Number of Master Interfaces:** `1` (connects to Zynq HP0)
   - Click **OK**

3. Connect:
   - `axi_dma_0 M_AXI_MM2S` → `smartconnect_0 S00_AXI`
   - `axi_dma_0 M_AXI_S2MM` → `smartconnect_0 S01_AXI`
   - `smartconnect_0 M00_AXI` → `zynq_ultra_ps_e_0 S_AXI_HPC0_FPD`
   - `clk_wiz_0 clk_out1` → `smartconnect_0 aclk`
   - `proc_sys_reset_0 interconnect_aresetn[0:0]` → `smartconnect_0 aresetn`
   - `clk_wiz_0 clk_out1` → `zynq_ultra_ps_e_0 saxihpc0_fpd_aclk`

### Step 6.9 — Temporarily Wire the DMA in Loopback (Phase 1)

Before we add the scalar multiplier, we want to confirm the DMA pipeline itself works.
Do this by connecting the DMA's output directly back to its input:

- `axi_dma_0 M_AXIS_MM2S` → `axi_dma_0 S_AXIS_S2MM`

> **Why?** This creates a hardware loopback: whatever Python sends comes straight back.
> If this test passes, any future failure is in the custom IP, not the DMA setup.
> This isolates bugs to one variable at a time.

Also connect the clock to the stream ports:
- `clk_wiz_0 clk_out1` → `axi_dma_0 m_axis_mm2s_aclk`
- `clk_wiz_0 clk_out1` → `axi_dma_0 s_axis_s2mm_aclk`

### Step 6.10 — Assign Memory Addresses

The PS accesses IP blocks by their memory address (like pointers in C). Vivado assigns
these automatically.

In the menu bar at the top of the block diagram editor, click:
**Window → Address Editor**

A table appears showing all the IP blocks and their assigned addresses. Click the
**"Auto Assign Address"** button (or press **Ctrl+Shift+A**).

Vivado will assign addresses to all slave interfaces. You do not need to note these
down — PYNQ reads them from the `.hwh` file automatically.

Close the Address Editor.

### Step 6.11 — Validate the Design

Press **F6** (or click the checkmark icon in the block diagram toolbar).

Vivado runs Design Rule Checks (DRC). Common results:

- **Green checkmark:** All good
- **Critical Warning (yellow):** Usually unconnected optional ports. Read the message.
  Most of the time these are DMA interrupt ports (`mm2s_introut`, `s2mm_introut`) which
  are safe to leave unconnected for now.
- **Error (red):** Must fix before continuing. Click the error for details.

If you see errors about unconnected clocks or resets, double-check all the connections
in Steps 6.4–6.8.

### Step 6.12 — Create the HDL Wrapper

The block diagram needs to be wrapped in a top-level RTL module before it can be
synthesized. Vivado does this automatically.

In the **Sources** panel (left side of Vivado), right-click on **`design_1`** (or
whatever your block design is named) → click **"Create HDL Wrapper"**.

In the dialog that appears:
- Select **"Let Vivado manage wrapper and auto-update"**
- Click **OK**

A new file `design_1_wrapper.v` appears in Sources. This is the top-level module.

---

## 7. Generate the Bitstream

This is the compilation step. Vivado converts your block diagram into a bitstream file
that can actually program the FPGA.

### Step 7.1 — Run "Generate Bitstream"

In the **Flow Navigator** panel on the left side of Vivado, click **"Generate Bitstream"**.

A dialog may ask if you want to run Synthesis and Implementation first — click **Yes**.

Vivado will now run three stages:
1. **Synthesis** (~5 minutes): Converts your block diagram into logical gates
2. **Implementation** (~10 minutes): Places those gates onto the physical FPGA fabric
3. **Bitstream** (~2 minutes): Packages the configuration into the `.bit` file

The progress bar shows which stage is running. **Total time: approximately 15-20 minutes.**
This is normal. Get a coffee.

### Step 7.2 — Find the Output Files

When bitstream generation finishes, a dialog asks what to do. Click **"Cancel"** (we
don't need to open the Hardware Manager).

You need two files for PYNQ:

**The bitstream file (`.bit`):**
```bash
find /home/cpsl/kria_dma_pipeline/ -name "*.bit"
```
It will be in a path like: `.../kria_dma_pipeline.runs/impl_1/design_1_wrapper.bit`

**The hardware handoff file (`.hwh`):**
```bash
find /home/cpsl/kria_dma_pipeline/ -name "*.hwh"
```
It will be in a path like: `.../kria_dma_pipeline.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh`

> **What is the HWH file?** The Hardware Handoff file is automatically generated by
> Vivado alongside the bitstream. It contains a complete map of every IP block in your
> design: their names, types, memory addresses, and how they connect. PYNQ reads this
> file to automatically discover your hardware without you writing device tree files.

---

## 8. Write the Scalar Multiplier in Vitis HLS

Now we leave Vivado and switch to **Vitis HLS** to build the custom IP. We will return
to Vivado later to replace the loopback wire with this IP.

### What We Are Building

A C++ function that:
1. Reads one 32-bit value from its AXI-Stream input
2. Doubles the lower 16 bits (the "real" part) and the upper 16 bits (the "imaginary" part)
3. Writes the result to its AXI-Stream output

The function processes one 32-bit word per clock cycle, running continuously.

### Step 8.1 — Open a Terminal and Source Vitis HLS

```bash
source /tools/Xilinx/Vitis_HLS/2025.2/settings64.sh
vitis &
```

This opens the **Vitis Unified IDE** — the same VS Code-based interface you have already seen.

### Step 8.2 — Write the Source Files

Before creating the project, write the C++ files. In the Vitis terminal (or any terminal):

```bash
mkdir -p /home/cpsl/scalar_mult/src
```

**Write the main design file:**
```bash
cat > /home/cpsl/scalar_mult/src/scalar_mult.cpp << 'EOF'
// scalar_mult.cpp
// This is the hardware function we are synthesizing.
// It reads one 32-bit AXI-Stream word, doubles both 16-bit halves, and writes the result.

#include "ap_int.h"        // Vitis HLS arbitrary-width integer types
#include "ap_axi_sdata.h"  // Vitis HLS AXI-Stream data structure
#include "hls_stream.h"    // Vitis HLS stream (FIFO) type

// ap_axis<32,1,1,1> = a 32-bit AXI-Stream word with standard sideband signals:
//   data[31:0]  — our payload
//   keep[3:0]   — byte enable (which bytes are valid)
//   strb[3:0]   — byte strobe
//   last        — signals end of packet (the DMA uses this)
typedef ap_axis<32, 1, 1, 1> stream_word;

void scalar_mult(hls::stream<stream_word> &in_s,
                 hls::stream<stream_word> &out_s) {

    // These pragmas tell Vitis HLS how to generate the hardware interface.
    // "axis" = AXI4-Stream protocol (the streaming protocol the DMA speaks)
    // "ap_ctrl_none" = no start/done control signals; IP runs continuously
    #pragma HLS INTERFACE axis         port=in_s
    #pragma HLS INTERFACE axis         port=out_s
    #pragma HLS INTERFACE ap_ctrl_none port=return

    stream_word w;
    in_s.read(w);   // Read one word from the input stream

    // The 32-bit data word is split: bits[15:0] = real, bits[31:16] = imaginary
    // ap_int<16> interprets a 16-bit slice as a signed integer
    ap_int<16> real_in = w.data.range(15,  0);
    ap_int<16> imag_in = w.data.range(31, 16);

    // Multiply both halves by 2
    w.data.range(15,  0) = real_in * 2;
    w.data.range(31, 16) = imag_in * 2;

    // The 'last' signal passes through automatically (it's part of the stream_word struct)
    // 'last' tells the DMA receiver "this is the final word in the packet"

    out_s.write(w);  // Write the modified word to the output stream
}
EOF
```

**Write the testbench file:**
```bash
cat > /home/cpsl/scalar_mult/src/scalar_mult_tb.cpp << 'EOF'
// scalar_mult_tb.cpp
// This is a software test. It runs the function in regular C++ to verify correctness.
// Vitis HLS will run this during "C Simulation" before synthesizing hardware.

#include <iostream>
#include "ap_int.h"
#include "ap_axi_sdata.h"
#include "hls_stream.h"

typedef ap_axis<32, 1, 1, 1> stream_word;

// Forward declaration of the function we are testing
void scalar_mult(hls::stream<stream_word> &in_s,
                 hls::stream<stream_word> &out_s);

int main() {
    hls::stream<stream_word> in_s, out_s;
    int errors = 0;

    // Send 8 test values: 0, 1, 2, ..., 7
    // We put each value in the lower 16 bits (real part); upper 16 bits = 0
    for (int i = 0; i < 8; i++) {
        stream_word w;
        w.data = (ap_uint<32>)i;    // value i in bits [15:0], 0 in bits [31:16]
        w.last = (i == 7) ? 1 : 0;  // mark the last word in the packet
        w.keep = 0xF;                // all 4 bytes valid
        w.strb = 0xF;
        in_s.write(w);
    }

    // Call the function once per word
    // (because ap_ctrl_none means the function runs once and processes one word)
    for (int i = 0; i < 8; i++) {
        scalar_mult(in_s, out_s);
    }

    // Check results: real_out should be i*2, imag_out should still be 0
    for (int i = 0; i < 8; i++) {
        stream_word result = out_s.read();
        ap_int<16> real_out = result.data.range(15, 0);
        ap_int<16> imag_out = result.data.range(31, 16);

        if (real_out != i * 2 || imag_out != 0) {
            std::cout << "FAIL at i=" << i
                      << "  real_out=" << real_out
                      << "  imag_out=" << imag_out << std::endl;
            errors++;
        }
    }

    if (errors == 0)
        std::cout << "C simulation PASS — all 8 values correct" << std::endl;

    return errors;  // returning non-zero causes Vitis HLS to report the sim as failed
}
EOF
```

Verify both files exist:
```bash
ls /home/cpsl/scalar_mult/src/
# Should show: scalar_mult.cpp  scalar_mult_tb.cpp
```

### Step 8.3 — Set the Workspace

Back in Vitis, click **"Set Workspace"** in the left panel (or under "Get Started").

Navigate to `/home/cpsl/scalar_mult` and click **Open**.

### Step 8.4 — Create a New HLS Component

Click **"New HLS Component"** from the Welcome screen (center panel, under "HLS Development").

The wizard opens:

---

**Screen 1 — Name and Location:**
- **Component name:** `scalar_mult`
- **Component location:** `/home/cpsl/scalar_mult` (should auto-fill)
- Click **Next**

---

**Screen 2 — Configuration File:**
- Select **"Empty File"** (the first radio button)
- Leave the config name as `hls_config`
- Click **Next**

---

**Screen 3 — Source Files:**

This screen has two sections.

*Design Files (top half):*
1. Click the **`+`** button in the Design Files section
2. Navigate to `/home/cpsl/scalar_mult/src/scalar_mult.cpp`
3. Select it, click **OK**
4. The file appears in the list. Look for a **"Top Function"** column next to it.
   Click that cell and type: `scalar_mult`
   > The top function is the C++ function that becomes the hardware module's entry
   > point. Everything this function calls gets synthesized into hardware.

*Testbench Files (bottom half):*
1. Click the **`+`** button in the Testbench Files section
2. Navigate to `/home/cpsl/scalar_mult/src/scalar_mult_tb.cpp`
3. Select it, click **OK**
   > Do NOT set a top function for the testbench. It's just a regular C++ program
   > that tests the design — it doesn't become hardware.

Click **Next**

---

**Screen 4 — Hardware:**

Two things to configure:

*Part:*
1. Click the **`...`** browse button next to the part field
2. A parts selector opens. In the search box, type: `xck26`
3. Select **`xck26-sfvc784-2LV-c`** from the list
4. Click **OK**

*Clock period:*
- Find the "Clock Period" field and type: `5`
  (Units are nanoseconds. 5 ns = 200 MHz, matching our Vivado clock)

Click **Next**

---

**Screen 5 — Settings:**
Leave everything at defaults. Do not change anything here.

Click **Next**

---

**Screen 6 — Summary:**
Confirm:
- Part: `xck26-sfvc784-2LV-c`
- Clock: 5 ns
- Top function: `scalar_mult`

Click **Finish**

---

### Step 8.5 — Run C Simulation

After Finish, Vitis opens the component workspace. On the left side, look for the
**"FLOW"** section (you may need to scroll down in the left panel). It shows:

```
FLOW
  Component: scalar_mult
  ▼ C SIMULATION
      Run
      Debug
      Reports
  ▼ C SYNTHESIS
      Run
      Reports
  ▼ C/RTL CO-SIMULATION
      Run
  ▼ IMPLEMENTATION
      Run
```

Click **Run** under **C SIMULATION**.

This compiles your C++ with a standard compiler and runs `scalar_mult_tb.cpp`.
It takes about 30 seconds.

**Look at the output panel at the bottom of the window.** You should see:
```
C simulation PASS — all 8 values correct
C-simulation finished successfully
```

> **If you see FAIL:** The bug is in your C++ code. Fix it here before continuing.
> It is much faster to find bugs in software simulation than in actual hardware.
> Common mistakes: wrong bit range in `data.range()`, forgetting to write to `out_s`.

### Step 8.6 — Run C Synthesis

Click **Run** under **C SYNTHESIS**.

This is the real compilation step — it converts your C++ into Verilog hardware
description code. Takes 1-3 minutes.

When it finishes, click **Reports** under C SYNTHESIS to see the synthesis summary.

**What to look at in the report:**

| Report item | What it means | What you want to see |
|---|---|---|
| Timing — Estimated (ns) | How fast the generated circuit can run | Less than 5.000 ns |
| Utilization — LUT | Logic look-up tables used | Small number (dozens) |
| Utilization — FF | Flip-flops (registers) used | Small number |
| Latency (cycles) | How many clock cycles per output | 1-2 for this design |

If "Estimated" timing shows more than 5.000 ns (a timing violation), add this line
at the very top of the function body in scalar_mult.cpp:
```cpp
#pragma HLS PIPELINE II=1
```
Then re-run synthesis. This tells HLS to pipeline the circuit for maximum throughput.

### Step 8.7 — Export the IP

Click **Run** under **IMPLEMENTATION**.

This packages the synthesized hardware into a format Vivado understands as an IP block.
Takes about 1 minute.

When it finishes, find the output directory:
```bash
find /home/cpsl/scalar_mult/ -type d -name "ip" 2>/dev/null
```

The output will be something like:
```
/home/cpsl/scalar_mult/scalar_mult/hls/impl/ip
```

**Write this path down** — you will need it in the next section.

---

## 9. Add the HLS IP to Vivado

Switch back to Vivado with your `kria_dma_pipeline` project open.

### Step 9.1 — Add the IP Repository

You need to tell Vivado where to find your new IP block.

1. In the top menu bar, click **Tools → Settings**
2. In the Settings dialog, look at the left tree. Click **IP → Repository**
3. Click the **`+`** (plus) button on the right side
4. Navigate to the `ip` directory you found above
   (e.g., `/home/cpsl/scalar_mult/scalar_mult/hls/impl/ip`)
5. Click **Select**
6. Vivado shows a message: *"1 IP definitions were found in the repository"*
7. Click **OK** → **OK** to close Settings

### Step 9.2 — Remove the Loopback Wire

Open your block diagram (`design_1` or `kria_bd`).

Find the direct wire you drew in Step 6.9 connecting:
`axi_dma_0 M_AXIS_MM2S` → `axi_dma_0 S_AXIS_S2MM`

**Click on that wire to select it** (it highlights blue when selected), then press the
**Delete** key. The wire disappears.

### Step 9.3 — Add the Scalar Multiplier IP

1. Right-click on the canvas → **Add IP**
2. In the search box, type `scalar_mult`
3. Your IP block should appear in the results with a name like "scalar_mult_v1_0" or similar
4. Double-click it to place it on the canvas

A new block appears. It will have these ports:
- `in_s` — AXI-Stream input
- `out_s` — AXI-Stream output
- `ap_clk` — clock input
- `ap_rst_n` — active-low reset input

### Step 9.4 — Connect the Scalar Multiplier

Make these four connections:

| From | To |
|---|---|
| `axi_dma_0` port `M_AXIS_MM2S` | `scalar_mult_0` port `in_s` |
| `scalar_mult_0` port `out_s` | `axi_dma_0` port `S_AXIS_S2MM` |
| `clk_wiz_0` port `clk_out1` | `scalar_mult_0` port `ap_clk` |
| `proc_sys_reset_0` port `peripheral_aresetn[0:0]` | `scalar_mult_0` port `ap_rst_n` |

> **Why no `ap_start` / `ap_done` ports?** Because we used `#pragma HLS INTERFACE ap_ctrl_none`
> in the C++ code. This tells HLS not to generate any control handshake ports — the IP
> processes data continuously as long as the input stream has data. This matches how
> AXI-Stream works: data flows whenever `tvalid` and `tready` are both high.

### Step 9.5 — Validate Again

Press **F6** to run Design Rule Checks again. Resolve any errors. Critical Warnings
about unconnected optional DMA interrupt ports are safe to ignore.

### Step 9.6 — Re-generate the Bitstream

In the Flow Navigator, click **"Generate Bitstream"** again.

This re-runs synthesis, implementation, and bitstream generation with the new IP in place.
Same 15-20 minutes.

### Step 9.7 — Find the New Output Files

```bash
find /home/cpsl/kria_dma_pipeline/ -name "*.bit"
find /home/cpsl/kria_dma_pipeline/ -name "*.hwh"
```

Note the paths to both files.

---

## 10. Deploy to the Kria Board

### Step 10.1 — Create a Directory on the Kria

SSH into the Kria and create a folder for your overlays:
```bash
ssh ubuntu@<kria_ip>
mkdir -p /home/ubuntu/my_overlays
```

### Step 10.2 — Copy the Files from Your PC to the Kria

Back on your **development PC** terminal:

```bash
# Copy the bitstream — rename it to something descriptive
scp /path/to/design_1_wrapper.bit  ubuntu@<kria_ip>:/home/ubuntu/my_overlays/kria_scalar_mult.bit

# Copy the HWH — MUST have the same base name as the .bit file
scp /path/to/design_1.hwh          ubuntu@<kria_ip>:/home/ubuntu/my_overlays/kria_scalar_mult.hwh
```

Replace `/path/to/` with the actual paths you found in Step 9.7.

> **The base name rule:** PYNQ finds the `.hwh` file by taking the `.bit` path and
> replacing `.bit` with `.hwh`. So if your bitstream is `kria_scalar_mult.bit`, PYNQ
> looks for `kria_scalar_mult.hwh` in the same folder. If the names don't match, PYNQ
> cannot discover your hardware and will throw an error.

### Step 10.3 — Verify the Files Arrived

```bash
ssh ubuntu@<kria_ip>
ls -lh /home/ubuntu/my_overlays/
```

You should see both files with the same base name:
```
-rw-r--r-- 1 ubuntu ubuntu  12M Mar 30 14:22 kria_scalar_mult.bit
-rw-r--r-- 1 ubuntu ubuntu 128K Mar 30 14:22 kria_scalar_mult.hwh
```

---

## 11. Run the Python Test

There are two ways to run the test: in JupyterLab (easier) or as a script via SSH
(more reliable for debugging).

### Method A — SSH Script

Create the test script on the Kria:

```bash
# SSH into the Kria
ssh ubuntu@<kria_ip>

# Create the test script
cat > /home/ubuntu/test_scalar_mult.py << 'EOF'
# test_scalar_mult.py
# Sends [0, 1, 2, ..., 1023] through the DMA pipeline and checks that the
# scalar_mult IP doubled every value. Prints a table of the first 8 samples.

import subprocess
import numpy as np
from pynq import Overlay, allocate

# ── Step 1: Release the PL lock ──────────────────────────────────────────────
# When the Kria boots, the OS loads a default FPGA application that holds a
# lock on the PL fabric. If we try to load our own overlay without releasing
# this lock, Overlay() will hang forever and never return.
subprocess.run(["sudo", "xmutil", "unloadapp"], capture_output=True)

# ── Step 2: Load the overlay ──────────────────────────────────────────────────
# This programs the PL fabric with our bitstream and reads the .hwh file
# to discover all IP blocks automatically.
print("Loading overlay...")
ol = Overlay("/home/ubuntu/my_overlays/kria_scalar_mult.bit")
dma = ol.axi_dma_0  # PYNQ names this after the block diagram block name
print("Overlay loaded.")

# ── Step 3: Allocate DMA-visible buffers ──────────────────────────────────────
# Regular Python/NumPy arrays live in virtual memory that the DMA cannot access.
# pynq.allocate() creates buffers in physically contiguous memory that the DMA
# can address directly. Think of these like numpy arrays with superpowers.
N = 1024
input_buffer  = allocate(shape=(N,), dtype=np.int32)
output_buffer = allocate(shape=(N,), dtype=np.int32)

# ── Step 4: Fill the input buffer ────────────────────────────────────────────
# We send [0, 1, 2, ..., 1023]. Each value goes into bits[15:0] of a 32-bit word.
# Bits[31:16] (the imaginary part) stay zero.
input_buffer[:] = np.arange(N, dtype=np.int32)

# ── Step 5: The DMA transfer ──────────────────────────────────────────────────
# These four operations must happen in exactly this order.

# flush() forces the CPU to write its cache to actual RAM.
# The DMA reads from RAM, not CPU cache. Without this, the DMA reads stale
# (possibly all-zero) data from RAM instead of what we just wrote.
input_buffer.flush()

# Arm the receive channel FIRST before starting the send.
# If you start send first, the scalar_mult IP begins outputting data before
# the receive channel is ready and the first samples get dropped.
dma.recvchannel.transfer(output_buffer)
dma.sendchannel.transfer(input_buffer)

# Wait for both transfers to complete
dma.sendchannel.wait()
dma.recvchannel.wait()

# invalidate() forces the CPU to discard its stale cache for output_buffer.
# The DMA wrote results to RAM, but the CPU's cache still has the old (zero)
# values. Without invalidate(), reading output_buffer returns zeros.
output_buffer.invalidate()

# ── Step 6: Check the results ────────────────────────────────────────────────
# The scalar_mult IP doubles bits[15:0] (real) and bits[31:16] (imaginary).
# Since our imaginary parts are zero, we just check the real (lower 16 bits).
real_out = (output_buffer & 0x0000FFFF).astype(np.int16).astype(np.int32)
expected = np.arange(N, dtype=np.int32) * 2

# Print a table of the first 8 values so you can visually confirm the math
print()
print("┌─────┬─────────┬──────────┬──────────┬────────┐")
print("│ idx │  input  │ expected │   got    │ match? │")
print("├─────┼─────────┼──────────┼──────────┼────────┤")
for i in range(8):
    match = "✓" if real_out[i] == expected[i] else "✗ FAIL"
    print(f"│ {i:<3} │  {input_buffer[i]:<6} │  {expected[i]:<7} │  {real_out[i]:<7} │ {match}  │")
print("└─────┴─────────┴──────────┴──────────┴────────┘")

# Check all 1024 values
passed = np.array_equal(real_out, expected)
print()
if passed:
    print(f"✓ All {N} samples correct. PASS")
else:
    diff_idx = np.where(real_out != expected)[0]
    print(f"✗ FAIL — {len(diff_idx)} mismatches out of {N} samples")
    print(f"  First bad index: [{diff_idx[0]}]")
    print(f"    input:    {input_buffer[diff_idx[0]]}")
    print(f"    expected: {expected[diff_idx[0]]}")
    print(f"    got:      {real_out[diff_idx[0]]}")

# ── Step 7: Release the overlay ───────────────────────────────────────────────
ol.free()
EOF
```

Run it with root permissions (required because PYNQ accesses hardware):
```bash
sudo -i
python3 /home/ubuntu/test_scalar_mult.py
```

### What You Should See

```
Loading overlay...
Overlay loaded.

┌─────┬─────────┬──────────┬──────────┬────────┐
│ idx │  input  │ expected │   got    │ match? │
├─────┼─────────┼──────────┼──────────┼────────┤
│ 0   │  0      │  0       │  0       │ ✓      │
│ 1   │  1      │  2       │  2       │ ✓      │
│ 2   │  2      │  4       │  4       │ ✓      │
│ 3   │  3      │  6       │  6       │ ✓      │
│ 4   │  4      │  8       │  8       │ ✓      │
│ 5   │  5      │  10      │  10      │ ✓      │
│ 6   │  6      │  12      │  12      │ ✓      │
│ 7   │  7      │  14      │  14      │ ✓      │
└─────┴─────────┴──────────┴──────────┴────────┘

✓ All 1024 samples correct. PASS
```

### Method B — JupyterLab

If you prefer a notebook environment:

1. Open JupyterLab at `http://<kria_ip>:9090/lab`
2. Click **File → New → Notebook**
3. In the first cell, paste the entire Python script above (without the `cat << 'EOF'`
   wrapper, just the Python code between the EOF markers)
4. Click **Run** (▶ button) or press **Shift+Enter**

> ⚠️ If you have multiple notebooks open and both try to use the DMA, they will
> conflict. Close other notebooks before running the test.

---

## 12. Troubleshooting Reference

### `Overlay()` hangs forever and never returns

**Cause:** The Kria's default `k26-starter-kits` firmware app is loaded and holds a
lock on the PL fabric.

**Fix:** Run this before every overlay load:
```bash
sudo xmutil unloadapp
```
Or add it to your script (the test script above already does this):
```python
import subprocess
subprocess.run(["sudo", "xmutil", "unloadapp"], capture_output=True)
```

### All output values are zero

**Cause:** Missing `flush()` or `invalidate()`. The DMA bypasses the CPU cache.
If you do not flush before sending, the DMA reads the old (zero) values from RAM.
If you do not invalidate after receiving, Python reads the old (zero) cache instead
of what the DMA wrote.

**Fix:** Make sure your script uses all four operations in the correct order:
```python
input_buffer.flush()
dma.recvchannel.transfer(output_buffer)
dma.sendchannel.transfer(input_buffer)
dma.sendchannel.wait()
dma.recvchannel.wait()
output_buffer.invalidate()
```

### Output values match input (not doubled) — scalar_mult not applied

**Cause:** The loopback wire from Phase 1 was never removed, or the HLS IP is not
connected in the block diagram.

**Fix:** Open the block diagram. Confirm that `scalar_mult_0` is present and:
- `axi_dma_0 M_AXIS_MM2S` → `scalar_mult_0 in_s`
- `scalar_mult_0 out_s` → `axi_dma_0 S_AXIS_S2MM`

Also verify you re-generated the bitstream AND deployed the new `.bit` and `.hwh` to
the Kria (it is easy to forget either of these).

### First samples are correct but later ones are wrong or missing

**Cause:** Send channel was started before receive channel. The IP outputted its first
samples before the receive channel was armed, and they were lost.

**Fix:** Always transfer the receive channel first:
```python
dma.recvchannel.transfer(output_buffer)   # FIRST
dma.sendchannel.transfer(input_buffer)    # SECOND
```

### "IP not found" when adding to Vivado block diagram

**Cause:** The IP repository was not added in Vivado, or was added at the wrong path.

**Fix:**
1. In a terminal: `find /home/cpsl/scalar_mult/ -type d -name "ip"` — note the exact path
2. In Vivado: **Tools → Settings → IP → Repository → +** → select that exact `ip/` folder
3. If the IP still does not appear, try: right-click in the Catalog panel → **Refresh IP Catalog**

### NumPy crash on PYNQ import

**Cause:** PYNQ installer upgraded NumPy to 2.x which is incompatible.

**Fix:**
```bash
pip install "numpy<2" --force-reinstall
```
Then restart the Jupyter kernel (Kernel → Restart Kernel).

### C Simulation fails in Vitis HLS

**Cause:** Bug in the C++ code or testbench. Common mistakes:
- Calling `scalar_mult()` the wrong number of times (must match number of writes to `in_s`)
- Wrong bit range: `data.range(15, 0)` not `data.range(0, 15)` (HLS uses descending range)
- Not writing to `out_s` before the testbench tries to read from it

**Fix:** Add `std::cout` print statements inside `scalar_mult()` to see what is happening.
Rerun C Simulation after each fix.

### Vivado Validate Design shows critical warning about DMA interrupt ports

**Symptom:** Warning says `mm2s_introut` or `s2mm_introut` are unconnected.

**This is safe to ignore** for our design. These are interrupt output ports that notify
the PS when a DMA transfer completes. We use polling (`wait()`) instead of interrupts,
so these ports are not needed.

### `dpkg lock` error during PYNQ install

```
E: Could not get lock /var/lib/dpkg/lock-frontend
```

**Fix:**
```bash
sudo systemctl stop unattended-upgrades
sudo bash install.sh -b KR260   # re-run installer, it continues where it stopped
```

---

## Appendix A — The Four Non-Negotiable DMA Rules

Every time you use the DMA in Python, these four rules apply. Breaking any one of them
causes silent data corruption that looks like "the hardware is broken."

```
Rule 1: xmutil unloadapp  — before every Overlay() load, or it hangs
Rule 2: flush()           — before DMA send, or DMA reads stale RAM
Rule 3: recv before send  — arm receive first, or first samples are lost
Rule 4: invalidate()      — after DMA receive, or Python reads stale cache
```

## Appendix B — Complete DMA Boilerplate Template

Copy-paste this every time you write a new DMA test:

```python
import subprocess, numpy as np
from pynq import Overlay, allocate

# Rule 1
subprocess.run(["sudo", "xmutil", "unloadapp"], capture_output=True)

ol  = Overlay("/home/ubuntu/my_overlays/YOUR_DESIGN.bit")
dma = ol.axi_dma_0

N    = 1024
ibuf = allocate(shape=(N,), dtype=np.int32)
obuf = allocate(shape=(N,), dtype=np.int32)

ibuf[:] = YOUR_INPUT_DATA_HERE

# Rules 2, 3, 4
ibuf.flush()
dma.recvchannel.transfer(obuf)   # Rule 3: recv first
dma.sendchannel.transfer(ibuf)
dma.sendchannel.wait()
dma.recvchannel.wait()
obuf.invalidate()                # Rule 4

# obuf now contains the output
ol.free()
```

## Appendix C — AXI-Stream 32-bit Data Packing

Our 32-bit stream word is split into two 16-bit signed integers:

```
  bit 31              bit 16  bit 15               bit 0
  ┌─────────────────────────┬─────────────────────────┐
  │  imaginary (XN_IM)      │  real (XN_RE)           │
  └─────────────────────────┴─────────────────────────┘
```

**Pack real + imaginary into a 32-bit buffer:**
```python
buf[:] = (imag.astype(np.uint16).astype(np.int32) << 16) | \
          real.astype(np.uint16).astype(np.int32)
```

**Unpack real and imaginary from a 32-bit buffer:**
```python
real_out = (buf & 0x0000FFFF).astype(np.int16).astype(np.float32)
imag_out = ((buf >> 16) & 0x0000FFFF).astype(np.int16).astype(np.float32)
```

## Appendix D — Quick Command Reference

**Development PC — launch tools:**
```bash
source /tools/Xilinx/Vivado/2025.2/settings64.sh && vivado &
source /tools/Xilinx/Vitis_HLS/2025.2/settings64.sh && vitis &
```

**Development PC — find Vivado output files:**
```bash
find /home/cpsl/<project>/ -name "*.bit"   # bitstream
find /home/cpsl/<project>/ -name "*.hwh"   # hardware handoff
```

**Development PC — copy files to Kria:**
```bash
scp design_1_wrapper.bit  ubuntu@<kria_ip>:/home/ubuntu/my_overlays/mydesign.bit
scp design_1.hwh          ubuntu@<kria_ip>:/home/ubuntu/my_overlays/mydesign.hwh
```

**Kria — run a Python script as root:**
```bash
sudo -i
python3 /home/ubuntu/your_script.py
```

**Kria — check what FPGA app is loaded:**
```bash
xmutil listapps
```

**Kria — release the PL lock:**
```bash
sudo xmutil unloadapp
```

**Kria — find the board's IP address:**
```bash
hostname -I
```

## Appendix E — Key Reference Documentation

| Resource | URL |
|---|---|
| KR260 Ubuntu 22.04 Boot Guide | https://xilinx.github.io/kria-apps-docs/kr260/linux_boot/ubuntu_22_04 |
| Kria PYNQ GitHub | https://github.com/Xilinx/Kria-PYNQ |
| Vitis HLS User Guide (UG1399) | https://docs.amd.com/r/en-US/ug1399-vitis-hls |
| AXI DMA Product Guide (PG021) | https://docs.amd.com/r/en-US/pg021_axi_dma |
| Whitney Knitter KR260 Tutorial Series | https://www.hackster.io/whitney-knitter |
| PYNQ Community Forum | https://discuss.pynq.io |