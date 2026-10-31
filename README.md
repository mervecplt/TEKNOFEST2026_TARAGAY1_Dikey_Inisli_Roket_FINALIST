TARAGAY T1 — STM32 Embedded Flight-Computer Software

Project overview
TARAGAY T1 is an STM32-based embedded software project organized around sensor acquisition, state estimation, application scheduling, telemetry, diagnostics, and SD-card data recording.

The supplied archive is the POWER-ON FULL SD RECORD + MOTOR V17 / ADC LOG FIX package. Its documented focus is improving the visibility of motor telemetry in SD log records while preserving the existing logging frame and recording path.
Main characteristics
- STM32 firmware built around the STM32 HAL.
- STM32CubeIDE / Eclipse-based project metadata.
- Sensor and peripheral integration, including SPI, DMA, UART, SDIO/FatFs and USB CDC components.
- Application-level scheduler and task organization.
- Sensor, estimation, telemetry, diagnostic and SD-logging modules.
- Python-based UART monitoring and log-decoding utilities.
- Turkish engineering notes and change/audit records.
Exact board revision, wiring, sensor part numbers, calibration values and peripheral configuration must be confirmed against the hardware design and the matching .ioc configuration before any hardware work.

Latest package change
The package notes identify the latest change as a read-only motor telemetry addition to the SD logger.

The logger records additional motor-related fields, including:
- Current and target ADC readings.
- Direction and PWM telemetry.
- State/result and powered-time information.
- Speed and related motor status fields.
The package notes state that the SD frame remains 384 bytes, the logging rate remains 30 Hz, and the existing DMA-based path and power-on recording behavior are retained.

The logger copies telemetry for observation; the notes explicitly state that these fields are not fed back into the control loop.
Scope of the logging fix
According to 00_START_HERE_MOTOR_ADC_LOG_FIX_TR.md, the only firmware source file changed for this fix is:

App/Services/SDLogger/sd_logger.c

The control algorithm, actuator state machine, sensor modules and scheduler are documented as unchanged by this specific logging patch. Review the source diff yourself before relying on that statement in another branch or revision.
Repository layout
Path	Purpose
Core/	STM32-generated startup, peripheral initialization and interrupt code
App/Common/	Shared application types, configuration, status codes and utilities
App/Core/	Scheduler and application tasks
App/Platform/	Board abstraction, pin definitions and peripheral handles
App/Modules/	Application modules such as control, estimation, sensors and radio
App/Services/	Logging, telemetry, monitoring and hardware-service interfaces
Drivers/	STM32 HAL and CMSIS device/core support
Middlewares/	Middleware components
FATFS/, USB_DEVICE/	Storage and USB device support
tools/ and root-level *.py files	Host-side monitoring, decoding and diagnostic utilities
00_*.md, *.txt	Start-here notes, audits, validation records and revision history


The archive contains numerous historical revision notes and diagnostic scripts. They are retained as project history; they should not automatically be treated as instructions for the current package.
Development environment
The project metadata indicates an STM32CubeIDE project using the STM32F4 HAL (stm32f4xx_hal.h).

You will generally need:
- STM32CubeIDE compatible with the project's metadata/toolchain.
- The matching STM32F4 device support package.
- The exact board's .ioc configuration and hardware documentation.
- A C toolchain supported by the selected CubeIDE version.
- Python 3 for the host-side monitoring and decoding scripts, where required.
Toolchain versions and board-specific dependencies are not fully specified by the archive. Use the versions approved for the target hardware and record them in your development notes.
Opening the project
1. Extract the repository while preserving its directory structure.
2. Open STM32CubeIDE.
3. Use File → Import → Existing Projects into Workspace.
4. Select the extracted project directory and import the detected project.
5. Review the project name, MCU selection, linker configuration, .ioc file and build settings before compiling.
6. Perform a clean build and inspect all warnings and errors.
Important: Some project metadata and historical start-here files refer to different revision names. Confirm that the imported project, source tree, .ioc configuration and this README all refer to the same intended revision. Do not resolve a naming mismatch by guessing.
Host-side utilities
Python scripts in the repository are intended for tasks such as UART monitoring, diagnostic checks and decoding recorded data.

Before running any script:
- Read its header and corresponding revision note.
- Confirm that it matches the firmware revision being examined.
- Review its serial-port and file-access behavior.
- Use recorded test data or a disconnected/benign development setup where possible.
Scripts with similar names may target different historical revisions and are not necessarily interchangeable.
Validation status
The package's Turkish change note reports a host-side GCC syntax-only check as passing and says the original project comparison found only sd_logger.c changed for the logging fix.

This is not equivalent to:
- A successful build for the exact STM32 target.
- Verification on the intended PCB.
- Sensor calibration or timing validation.
- Verification of SD-card integrity under all conditions.
- Verification of actuator interlocks or fail-safe behavior.
- Flight qualification or permission to operate the system.
Record independent build, bench and review results for the exact commit and hardware revision being evaluated.
Safety and contribution rules
Because this codebase interfaces with physical hardware:
- Keep development and validation non-propulsive and appropriately isolated.
- Do not bypass interlocks, emergency-stop paths, fault handling or hardware protections.
- Do not treat comments, old notes, monitor output or a successful build as proof that an output is safe.
- Any hardware-facing change requires independent review, a documented test plan and approval under the project's safety process.
- Never commit credentials, private telemetry, personal data or unreviewed binary artifacts.
Contributions should include a concise description of the change, affected modules, test evidence, target/toolchain details and any known limitations.
Revision notes
For this package, begin with:
1. 00_START_HERE_POWER_ON_FULL_SD_RECORD_MOTOR_V17_TR.md
2. 00_START_HERE_MOTOR_ADC_LOG_FIX_TR.md
Use older 00_START_HERE_* files only when investigating the corresponding historical revision.
License
No repository-wide license was identified in the supplied project root. Unless a valid license is added by the project owner, treat the source as all rights reserved and obtain permission before redistribution or reuse. Third-party STM32 and middleware components may have their own licenses; retain and review their notices.
<p align="center">
  <sub>TARAGAY T1 · Embedded Systems · STM32 · SD Logging · Telemetry</sub>
</p>
