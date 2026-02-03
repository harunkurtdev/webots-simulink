# Troubleshooting Guide

This guide helps you diagnose and resolve common issues when working with the Webots-Simulink integration framework.

---

## Installation Issues

### MATLAB Cannot Find Webots Libraries

**Symptoms:**
- Error: `Undefined function 'wb_robot_init'`
- MATLAB crashes when starting controller

**Solutions:**

1. **Verify Webots installation path:**
   ```matlab
   % Check if Webots is in system PATH
   getenv('WEBOTS_HOME')
   ```

2. **Add Webots MATLAB library manually:**
   ```matlab
   % Windows
   addpath('C:\Program Files\Webots\lib\controller\matlab');

   % macOS
   addpath('/Applications/Webots.app/Contents/lib/controller/matlab');

   % Linux
   addpath('/usr/local/webots/lib/controller/matlab');
   ```

3. **Set environment variable:**
   ```bash
   # Linux/macOS
   export WEBOTS_HOME=/usr/local/webots

   # Windows (Command Prompt)
   set WEBOTS_HOME=C:\Program Files\Webots
   ```

### Webots Doesn't Recognize MATLAB Controller

**Symptoms:**
- Controller selection shows empty or error
- "Controller not found" message

**Solutions:**

1. Ensure MATLAB is installed and licensed
2. Check that controller file has `.m` extension
3. Verify the controller directory structure:
   ```
   controllers/
   └── your_controller/
       └── your_controller.m  # Must match folder name
   ```

---

## Simulink Connection Issues

### Simulink Model Won't Load

**Symptoms:**
- `open_system('simulink_control')` fails
- Error: "File not found"

**Solutions:**

1. **Check current MATLAB directory:**
   ```matlab
   pwd  % Print working directory
   cd(fileparts(which('simulink_control_app.m')))
   ```

2. **Verify model file exists:**
   ```matlab
   exist('simulink_control.slx', 'file')  % Should return 4
   ```

3. **Check Simulink license:**
   ```matlab
   license('test', 'Simulink')  % Should return 1
   ```

### Simulink Runs But No Data Transfer

**Symptoms:**
- Simulink model runs but robot doesn't move
- Sensor values always zero

**Solutions:**

1. **Verify sensor initialization:**
   ```matlab
   % Sensors must be enabled before use
   sensor = wb_robot_get_device('sensor_name');
   wb_sensor_enable(sensor, TIME_STEP);  % Don't forget this!
   ```

2. **Check TIME_STEP matching:**
   ```matlab
   % MATLAB TIME_STEP must match Webots basicTimeStep
   TIME_STEP = 16;  % Check world file for basicTimeStep value
   ```

3. **Confirm wb_robot_step is called:**
   ```matlab
   % Main simulation loop must include
   wb_robot_step(TIME_STEP);
   ```

---

## Simulation Performance Issues

### Simulation Runs Very Slowly

**Symptoms:**
- Low FPS in Webots
- Laggy response
- High CPU usage

**Solutions:**

| Issue | Solution |
|-------|----------|
| Complex physics | Increase `basicTimeStep` in world file |
| Too many sensors | Disable unused sensors |
| High-poly meshes | Use simpler bounding objects |
| Debug output | Remove `disp()` calls in loop |
| Simulink overhead | Use Simulink Accelerator mode |

**Optimize Simulink model:**
```matlab
% Use accelerator mode
set_param('simulink_control', 'SimulationMode', 'accelerator');
```

### Simulation Freezes or Crashes

**Symptoms:**
- Webots becomes unresponsive
- MATLAB shows "busy" indefinitely

**Solutions:**

1. **Check for infinite loops:**
   ```matlab
   % Ensure simulation loop has proper exit condition
   while wb_robot_step(TIME_STEP) ~= -1
       % Your code here
   end
   ```

2. **Memory leak prevention:**
   ```matlab
   % Clear large variables periodically
   clear temp_data;
   ```

3. **Reduce data logging:**
   ```matlab
   % Log only when necessary
   if mod(step_count, 100) == 0
       log_data(data);
   end
   ```

---

## Sensor Issues

### Sensor Returns Zero or NaN Values

**Symptoms:**
- All sensor readings are 0
- NaN values in sensor data

**Solutions:**

1. **Enable sensor before reading:**
   ```matlab
   gyro = wb_robot_get_device('gyro');
   wb_gyro_enable(gyro, TIME_STEP);
   wb_robot_step(TIME_STEP);  % Wait one step
   values = wb_gyro_get_values(gyro);  % Now read
   ```

2. **Check sensor name spelling:**
   ```matlab
   % Names are case-sensitive!
   device = wb_robot_get_device('Gyro');     % Might fail
   device = wb_robot_get_device('gyro');     % Correct
   ```

3. **Verify sensor exists in world file:**
   - Open `.wbt` file in text editor
   - Search for sensor definition
   - Confirm `name` field matches your code

### LiDAR Data Is Empty or Incorrect

**Symptoms:**
- `wb_lidar_get_range_image` returns empty array
- Range values are all maximum or zero

**Solutions:**

1. **Enable point cloud if needed:**
   ```matlab
   lidar = wb_robot_get_device('lidar');
   wb_lidar_enable(lidar, TIME_STEP);
   wb_lidar_enable_point_cloud(lidar);
   ```

2. **Check LiDAR orientation:**
   - Verify LiDAR is not facing a wall/obstacle too close
   - Check minimum/maximum range settings in world file

3. **Get correct array dimensions:**
   ```matlab
   horizontal_res = wb_lidar_get_horizontal_resolution(lidar);
   num_layers = wb_lidar_get_number_of_layers(lidar);
   ```

---

## Motor Control Issues

### Motors Don't Respond to Commands

**Symptoms:**
- `wb_motor_set_velocity` has no effect
- Robot remains stationary

**Solutions:**

1. **Set position to infinity for velocity control:**
   ```matlab
   motor = wb_robot_get_device('motor');
   wb_motor_set_position(motor, inf);  % Enable velocity mode
   wb_motor_set_velocity(motor, 10.0);
   ```

2. **Check motor limits:**
   ```matlab
   % Motor velocity is clamped to maxVelocity in world file
   % Verify your command is within limits
   ```

3. **Verify motor initialization:**
   ```matlab
   motor = wb_robot_get_device('left_motor');
   if motor == 0
       error('Motor not found! Check device name.');
   end
   ```

### Motor Oscillates or Is Unstable

**Symptoms:**
- Motor vibrates rapidly
- Position control overshoots repeatedly

**Solutions:**

1. **Tune PID gains:**
   - Reduce proportional gain (P)
   - Increase derivative gain (D)
   - Add small integral gain (I) for steady-state error

2. **Add velocity limits:**
   ```matlab
   max_vel = 5.0;  % rad/s
   commanded_vel = min(max_vel, max(-max_vel, raw_vel));
   wb_motor_set_velocity(motor, commanded_vel);
   ```

3. **Use lower control frequency:**
   - Increase TIME_STEP value
   - Add a control rate divider

---

## Control System Issues

### State-Space Model Doesn't Match Simulation

**Symptoms:**
- Controller designed in MATLAB doesn't work in Webots
- System behaves differently than expected

**Solutions:**

1. **Verify physical parameters:**
   - Check mass values in world file
   - Verify inertia calculations
   - Confirm friction coefficients

2. **Account for unmodeled dynamics:**
   - Add damping terms
   - Include actuator dynamics
   - Consider sensor delays

3. **Linearization point:**
   - Ensure operating point matches design assumptions
   - Check if system is near linearization point

### LQR Controller Is Unstable

**Symptoms:**
- System diverges instead of stabilizing
- Large oscillations that grow over time

**Solutions:**

1. **Check controllability:**
   ```matlab
   Co = ctrb(A, B);
   rank(Co)  % Should equal number of states
   ```

2. **Verify state estimation:**
   - Ensure all states are measured or estimated
   - Check observer gains if using state estimator

3. **Increase Q weights for problematic states:**
   ```matlab
   Q = diag([100, 10, 1000, 10]);  % Increase weights
   K = lqr(A, B, Q, R);
   ```

---

## Common Error Messages

| Error Message | Cause | Solution |
|---------------|-------|----------|
| `Device not found` | Wrong device name | Check spelling and case |
| `Sensor not enabled` | Missing enable call | Call `wb_*_enable()` first |
| `Cannot open model` | Model file missing | Check file path and name |
| `License checkout failed` | MATLAB license issue | Verify Simulink license |
| `Simulation timeout` | Infinite loop | Add proper exit condition |
| `Out of memory` | Memory leak | Clear unused variables |

---

## Getting Help

If you're still experiencing issues:

1. **Check the [FAQ](faq.md)** for common questions
2. **Review example code** in the `examples/` directory
3. **Search existing issues** on GitHub
4. **Open a new issue** with:
   - Operating system and versions (MATLAB, Webots)
   - Complete error message
   - Minimal code to reproduce the issue
   - What you've already tried

---

## Debug Tips

### Enable Verbose Output

```matlab
% Add debug prints to track execution
disp(['Step: ', num2str(step_count)]);
disp(['Sensor values: ', num2str(values)]);
```

### Log Data for Analysis

```matlab
% Create data log
data_log = [];

while wb_robot_step(TIME_STEP) ~= -1
    sensor_data = wb_gyro_get_values(gyro);
    data_log = [data_log; sensor_data'];
end

% Save for later analysis
save('simulation_log.mat', 'data_log');
```

### Test Components Individually

1. Test sensor reading without control
2. Test motor commands without sensor feedback
3. Test Simulink model in isolation
4. Combine components incrementally
