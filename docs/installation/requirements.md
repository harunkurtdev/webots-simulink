# Requirements

Before setting up the **Webots-Simulink Bridge**, ensure that your system meets the following requirements.

## Software Requirements
- **MATLAB & Simulink** (Recommended: R2021a or later)
- **Webots** (Recommended: Latest stable version)
- **C Compiler** compatible with MATLAB (e.g., MinGW, MSVC)

## MATLAB Toolboxes
Ensure that the following MATLAB toolboxes are installed:
- **MATLAB Coder** (for interfacing with external libraries)
- **Simulink** (for model-based design and simulation)
- **Embedded Coder** (optional, for advanced code generation)

## System Requirements
- **Operating System**: Windows 10/11, Ubuntu 20.04+ (Recommended)
- **RAM**: At least 8GB (16GB recommended for large simulations)
- **Processor**: Multi-core processor (Intel i5/i7 or AMD Ryzen recommended)
- **GPU**: Dedicated GPU recommended for better rendering in Webots

## Dependencies
The following dependencies should be installed and configured properly:
- **Webots API** (Ensure Webots' shared libraries are accessible by MATLAB)
- **MATLAB External Library Support** (Proper setup of `calllib` for Webots' functions)

Once these requirements are met, proceed to the [Setup Guide](setup.md) to configure the bridge.