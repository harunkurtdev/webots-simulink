# System Requirements

This page lists all software, hardware, and toolbox requirements for the Webots-Simulink Bridge.

---

## Software Requirements

| Software | Minimum Version | Recommended Version | Notes |
|----------|-----------------|---------------------|-------|
| MATLAB | R2020b | R2024a or later | Required for Simulink integration |
| Simulink | R2020b | R2024a or later | Included with MATLAB |
| Webots | R2022a | R2023a or later | Open-source robotics simulator |
| C Compiler | - | MinGW-w64 (Windows) / GCC (Linux) | Required for MEX compilation |

---

## MATLAB Toolboxes

The following MATLAB toolboxes are required or recommended:

| Toolbox | Required | Purpose |
|---------|----------|---------|
| **Simulink** | Yes | Block-diagram based modeling and simulation |
| **MATLAB Coder** | Yes | Generates C code from MATLAB functions for Webots API calls |
| **Control System Toolbox** | Recommended | State-space modeling, LQR design, and transfer functions |
| **Embedded Coder** | Optional | Optimized code generation for ROS 2 deployment |
| **ROS Toolbox** | Optional | Required for ROS 2 package export |

To check installed toolboxes, run this command in MATLAB:

```matlab
ver
```

---

## Operating System Support

| Operating System | Status | Notes |
|------------------|--------|-------|
| Windows 10/11 | Fully Supported | Tested with MSVC and MinGW compilers |
| Ubuntu 20.04 LTS | Fully Supported | Recommended for ROS 2 integration |
| Ubuntu 22.04 LTS | Fully Supported | Best compatibility with ROS 2 Humble |
| macOS 12+ | Partially Supported | Limited testing; some features may not work |

---

## Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **Processor** | Quad-core CPU (Intel i5 / AMD Ryzen 5) | 8-core CPU (Intel i7 / AMD Ryzen 7) |
| **RAM** | 8 GB | 16 GB or more |
| **Storage** | 10 GB free space | SSD with 20 GB free space |
| **GPU** | Integrated graphics | Dedicated GPU (NVIDIA/AMD) for better Webots rendering |

---

## Webots API Dependencies

The bridge requires access to Webots controller libraries. Ensure the following paths are accessible:

| Operating System | Default Library Path |
|------------------|---------------------|
| Windows | `C:\Program Files\Webots\lib\controller\matlab` |
| Linux | `/usr/local/webots/lib/controller/matlab` |
| macOS | `/Applications/Webots.app/Contents/lib/controller/matlab` |

---

## Compiler Configuration

MATLAB requires a compatible C compiler for MEX file compilation. To check your current compiler:

```matlab
mex -setup
```

### Recommended Compilers

| Operating System | Compiler | Installation |
|------------------|----------|--------------|
| Windows | MinGW-w64 | Install via MATLAB Add-Ons |
| Windows | Microsoft Visual C++ | Install Visual Studio Build Tools |
| Linux | GCC | `sudo apt install build-essential` |
| macOS | Xcode Command Line Tools | `xcode-select --install` |

---

## Next Steps

Once all requirements are met, proceed to the [Setup Guide](setup.md) to configure the Webots-Simulink Bridge.
