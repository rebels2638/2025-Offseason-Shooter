# Agent Protocol: Simulation & Testing

**Role:** You are a Software Engineer Agent responsible for testing FRC robot code.
**Objective:** Verify code changes by running simulations and analyzing the resulting log files.
**Tools:** A dedicated Python suite in `agent/tools/` for running sims and parsing logs.

---

## 1. Environment Setup (Execute Once)

Before running tests, ensure the Python environment is ready.

```bash
source venv/bin/activate

```

---

## 2. Testing Workflow

Follow this loop for every code change you need to verify.

### Step A: Configure Robot for Simulation
1.  Open `src/main/java/frc/robot/constants/Constants.java`.
2.  Set `public static final Mode currentMode = Mode.SIM;`.
3.  Set `public static final boolean agentMode = true;`.

### Step B: Define the Test Case
1.  Open `src/main/java/frc/robot/Robot.java`.
2.  Locate `autonomousInit()`.
3.  Assign your test command to `m_autonomousCommand` when `agentMode` is true.

**Pattern:**
```java
@Override
public void autonomousInit() {
    if (Constants.agentMode) {
        // COMMAND TO TEST GOES HERE
        m_autonomousCommand = new RunShooterFlywheel(5000); 
    } else {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    }
    // ...
}
```

**Tip:** Use `Logger.recordOutput("Test/MyMetric", value)` anywhere in the code (Subsystems, Commands, or the test command itself) to log specific data points for analysis. You should also check existing logging statements in the codebase if you are looking for specific keys to analyze.

### Step C: Run Simulation
Use the wrapper tool to run a headless simulation for a fixed duration. This automatically generates a `.wpilog` file in `agent/logs/`.

```bash
# Run sim for 15 seconds (default)
python3 agent/tools/sim_wrapper.py --duration 15
```

*Alternative (Manual):*
```bash
./gradlew simulateJava &
pid=$!
sleep 15
kill $pid
```

### Step D: Verify Results
Analyze the latest log file to determine if the test passed.

**1. List available logs:**
```bash
python3 agent/tools/list_logs.py
```

**2. Identify relevant keys:**
Use `list_keys.py` to find the exact key names in the log.
```bash
python3 agent/tools/list_keys.py --filter "Shooter"
```

**3. Analyze specific data:**
Use `log_tool.py` to query data from the latest log.

*   **Check average value:**
    ```bash
    python3 agent/tools/log_tool.py --mode avg --key "Test/MyMetric" --start 2.0 --end 14.0
    ```

*   **Check min/max values:**
    ```bash
    python3 agent/tools/log_tool.py --mode minmax --key "Test/MyMetric"
    ```

*   **Dump values to console:**
    ```bash
    python3 agent/tools/log_tool.py --mode values --key "Test/MyMetric"
    ```

*   **Check Driver Station state:**
    ```bash
    python3 agent/tools/log_tool.py --mode ds --start 5.0
    ```

---

## 3. Tool Reference

### `agent/tools/sim_wrapper.py`
*   **Usage:** `python3 agent/tools/sim_wrapper.py [--duration SECONDS]`
*   **Function:** Runs `./gradlew simulateJava`, waits, kills the process, and prepares the log.

### `agent/tools/log_tool.py`
*   **Usage:** `python3 agent/tools/log_tool.py --mode [MODE] --key [KEY] [OPTIONS]`
*   **Modes:**
    *   `timestamps`: Start/end time of the log/key.
    *   `values`: Raw values (supports `--limit`).
    *   `avg`: Average value (supports `--start`, `--end`).
    *   `minmax`: Minimum and maximum values.
    *   `deriv`: Rate of change (derivative).
    *   `ds`: Driver Station state.

### `agent/tools/list_keys.py`
*   **Usage:** `python3 agent/tools/list_keys.py [--file PATH] [--filter TEXT]`
*   **Function:** Lists all keys in the log file (default: latest). Use `--filter` to search for specific substrings (e.g., "Shooter").

### `agent/tools/open_in_viewer.py`
*   **Usage:** `python3 agent/tools/open_in_viewer.py`
*   **Function:** Opens the latest log in the **AdvantageScope** GUI application (macOS only).
*   **Note:** This tool is for **manual human inspection** only. Agents should use `log_tool.py` to verify data programmatically.
