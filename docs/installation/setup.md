# Setup Guide

This guide will help you configure the **Webots-Simulink Bridge** to enable seamless interaction between Webots and MATLAB/Simulink.

## 1. Install Webots
1. Download Webots (recomendly 2023a) from the [official website](https://cyberbotics.com/).
2. Follow the installation instructions for your operating system.
3. Verify the installation by running Webots and opening a sample world.

## 2. Install MATLAB & Required Toolboxes
1. Install **MATLAB & Simulink** (R2024a or later recommended).
2. Open MATLAB and ensure the following toolboxes are installed:
   - MATLAB Coder
   - Simulink
   - Embedded Coder (optional)
3. If a required toolbox is missing, install it using the MATLAB Add-On Manager.

## 3. Configure Webots Library in MATLAB
1. Add Webots' controller library path to MATLAB:
   ```matlab
   addpath('C:/Program Files/Webots/lib/controller/matlab')
   savepath
   ```
2. Load the Webots shared library in MATLAB:
   ```matlab
   loadlibrary('Controller', 'controller.h')
   ```
3. Verify the library functions:
   ```matlab
   libfunctions('Controller')
   ```
   If the list of functions appears, the library is successfully loaded.


## Next Steps
After setup, refer to the [Usage Guide](usage/connecting.md) to start running simulations with Webots and Simulink.
