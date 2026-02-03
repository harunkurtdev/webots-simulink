# Debugging Guide

This guide provides techniques for troubleshooting issues with the Webots-Simulink Bridge.

---

## Common Issues and Solutions

### Connection Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| Webots not responding | Controller not set to `<extern>` | Set robot controller to `<extern>` in Webots |
| MATLAB hangs on init | Webots not running | Start Webots before running MATLAB script |
| Simulation freezes | Time step mismatch | Ensure TIME_STEP matches in both applications |
| No sensor data | Sensors not enabled | Call `wb_*_enable()` for each sensor |

### Data Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| NaN values | Sensor read before initialization | Wait one TIME_STEP after enabling sensors |
| Zero motor output | Motor not configured | Set position to `inf` for velocity control |
| Incorrect units | Unit conversion error | Check Webots documentation for units |
| Delayed response | Buffering issues | Reduce TIME_STEP value |

---

## Debugging Tools in Simulink

### 1. Add Display Blocks

Use Display blocks to show signal values during simulation:

```
1. Drag Display block from Simulink Library Browser
2. Connect to the signal you want to monitor
3. Run simulation and observe values
```

### 2. Add Scope Blocks

Visualize signals over time:

```
1. Drag Scope block from Simulink Library Browser
2. Connect to signals (position, velocity, control output)
3. Double-click to open scope window during simulation
4. Use zoom and pan to analyze data
```

### 3. Use To Workspace Blocks

Log data for post-simulation analysis:

```matlab
% After simulation, analyze logged data
plot(out.position.Time, out.position.Data);
xlabel('Time (s)');
ylabel('Position (m)');
title('Robot Position vs Time');
```

---

## Step-by-Step Debugging

### Method 1: Single-Step Execution

Run the simulation one step at a time:

```matlab
% Initialize
wb_robot_init();
TIME_STEP = 16;

% Single step with debug output
while wb_robot_step(TIME_STEP) ~= -1
    % Read and display sensor values
    position = wb_gps_get_values(gps);
    fprintf('Step: Position = [%.4f, %.4f, %.4f]\n', position(1), position(2), position(3));

    % Check for anomalies
    if any(isnan(position))
        warning('NaN detected in position!');
        break;
    end

    % Manual pause for inspection
    pause(0.1);
end
```

### Method 2: Breakpoints in MATLAB Function Blocks

Set breakpoints inside MATLAB Function blocks:

1. Open the MATLAB Function block
2. Click on the line number to set a breakpoint
3. Run simulation in debug mode
4. Step through code using F10 (step over) or F11 (step into)

### Method 3: Conditional Breakpoints

Stop simulation when specific conditions are met:

```matlab
function y = fcn(sensor_value, threshold)
    y = sensor_value;

    % Debug: stop if value exceeds threshold
    if sensor_value > threshold
        keyboard;  % Opens debug mode
    end
end
```

---

## Solver Configuration Issues

### Verify Solver Settings

```matlab
% Check current solver settings
get_param('model_name', 'Solver')
get_param('model_name', 'FixedStep')
get_param('model_name', 'StopTime')

% Correct settings for Webots integration
set_param('model_name', 'Solver', 'ode4');
set_param('model_name', 'SolverType', 'Fixed-step');
set_param('model_name', 'FixedStep', '0.016');
set_param('model_name', 'StopTime', 'inf');
```

### Algebraic Loop Detection

If you see algebraic loop errors:

1. Open **Model Settings** → **Diagnostics** → **Solver**
2. Set "Algebraic loop" to "warning" temporarily
3. Identify the feedback loop causing the issue
4. Add a Unit Delay block to break the loop

---

## Signal Monitoring

### Create a Debug Subsystem

```matlab
% Create monitoring function
function debug_monitor(position, velocity, control)
    persistent step_count;
    if isempty(step_count)
        step_count = 0;
    end
    step_count = step_count + 1;

    % Log every 100 steps
    if mod(step_count, 100) == 0
        fprintf('Step %d:\n', step_count);
        fprintf('  Position: [%.3f, %.3f, %.3f]\n', position);
        fprintf('  Velocity: [%.3f, %.3f, %.3f]\n', velocity);
        fprintf('  Control:  [%.3f, %.3f, %.3f]\n', control);
    end
end
```

### Check Signal Dimensions

Verify signal dimensions match expected values:

```matlab
function y = check_dimensions(input, expected_size)
    actual_size = size(input);
    if ~isequal(actual_size, expected_size)
        error('Dimension mismatch: expected [%s], got [%s]', ...
            num2str(expected_size), num2str(actual_size));
    end
    y = input;
end
```

---

## Model Configuration Checklist

| Setting | Expected Value | How to Check |
|---------|----------------|--------------|
| Solver | Fixed-step | Model Settings → Solver |
| Solver Type | ode4 (Runge-Kutta) | Model Settings → Solver |
| Fixed Step Size | 0.016 | Model Settings → Solver |
| Stop Time | inf | Model Settings → Solver |
| Data Types | double | Signal Properties |
| Sample Time | 0.016 or inherited | Block Parameters |

---

## Performance Profiling

### Enable Simulink Profiler

1. Go to **Debug** → **Performance Advisor**
2. Run simulation with profiling enabled
3. Analyze results to identify slow blocks

### MATLAB Profiler

```matlab
% Profile MATLAB code
profile on
run_simulation();
profile off
profile viewer
```

---

## Error Messages Reference

| Error | Meaning | Solution |
|-------|---------|----------|
| `wb_robot_step returned -1` | Simulation ended or Webots closed | Restart Webots and reconnect |
| `Device not found` | Invalid device name | Check device name in Webots scene tree |
| `Algebraic loop detected` | Feedback without delay | Add Unit Delay block |
| `Index exceeds array bounds` | Array dimension mismatch | Verify signal dimensions |
| `Output dimensions mismatch` | Block output size incorrect | Check MATLAB Function block output |

---

## When Stuck

### Reset Everything

```matlab
% Clear MATLAB state
clear all;
close all;
clc;

% Reload library
bdclose all;

% Restart from clean state
wb_robot_init();
```

### Simplify the Model

1. Create a minimal test case with one sensor and one actuator
2. Verify basic communication works
3. Add components one at a time
4. Test after each addition

### Check Documentation

- [Webots Reference Manual](https://cyberbotics.com/doc/reference/index)
- [Simulink Documentation](https://www.mathworks.com/help/simulink/)
- [Project Examples](../examples/inverted-pendulum.md)

---

## Next Steps

- [ROS 2 Export](export-ros2.md): Deploy your debugged controller on real hardware
- [Troubleshooting](../troubleshooting.md): Additional common issues and solutions
