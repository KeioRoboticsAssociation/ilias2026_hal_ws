# Migration Plan: H753 → Lib/mavlink_hal Generated Code

## Executive Summary

**Goal**: Replace manual `motor_registry` system with auto-generated code from `Lib/mavlink_hal` YAML configurations.

**Benefits**:
- Configuration-driven development (YAML → C code)
- Easier to maintain and modify motor configs
- Consistent code generation across platforms
- Reduced manual coding errors
- Better documentation (config is self-documenting)

**Timeline**: Estimated 8 tasks

---

## Current Architecture Analysis

### Current System (Manual)

```
Core/Src/freertos.c
    ↓
    motor_registry_init()
    motor_registry_create_all_motors()
    motor_registry_update_all()
    ↓
App/motors/motor_registry.c (334 lines)
    ├─ Reads: App/config/motor_config.h (490 lines)
    │   ├─ SERVO_CONFIGS[]
    │   ├─ DC_MOTOR_CONFIGS[]
    │   ├─ ROBOMASTER_CONFIGS[]
    │   ├─ RS485_CONFIGS[]
    │   └─ MOTOR_INSTANCES[]
    └─ Creates controllers via factory functions
        ├─ motor_factory_create_servo()
        ├─ motor_factory_create_dc_motor()
        ├─ motor_factory_create_robomaster()
        └─ motor_factory_create_rs485()
            ↓
App/motors/{servo,dc,robomaster,rs485}_controller.c/h
    └─ Individual controller implementations
```

**Current Files in `/App`:**
```
App/
├── comm/
│   ├── mavlink_udp.c/h          (MAVLink UDP transport)
│   └── parameter_manager.c/h    (PID parameter system)
├── config/
│   └── motor_config.h           (490 lines - REPLACE with YAML)
├── hal/
│   └── hardware_manager.c/h     (Hardware abstraction - KEEP)
└── motors/
    ├── motor_interface.h        (KEEP - vtable interface)
    ├── motor_registry.c/h       (334 lines - REMOVE after migration)
    ├── servo_controller.c/h     (KEEP - implementation)
    ├── dc_controller.c/h        (KEEP - implementation)
    ├── robomaster_controller.c/h (KEEP - implementation)
    └── rs485_controller.c/h     (KEEP - implementation)
```

---

## Target Architecture (Generated)

### New System (Generated from YAML)

```
Core/Src/freertos.c
    ↓
    mavlink_gen_init_all_devices()
    mavlink_gen_update_all_devices()
    mavlink_gen_handle_message()
    ↓
Lib/mavlink_hal/config/generated/h753_full/
    ├── mavlink_generated_config.h
    ├── mavlink_generated_devices.c
    ├── mavlink_generated_handlers.c
    └── mavlink_generated_params.h
        ↓
    Includes and uses:
        App/motors/{servo,dc,robomaster,rs485}_controller.c/h
        App/hal/hardware_manager.c/h
```

**New Configuration Flow:**
```
1. Edit YAML: Lib/mavlink_hal/config/examples/h753_full.yaml
2. Generate: python generator/cli.py generate h753_full.yaml
3. Build: cmake --build build/
```

---

## Migration Strategy

### Phase 1: Fix Code Generator Templates

**Problem**: Generated templates don't match h753 controller API

**Current Generated Code (WRONG)**:
```c
rs485_controllers[i] = rs485_controller_create(&rs485_cfg);  // Expects pointer return
```

**Actual API**:
```c
error_code_t rs485_controller_create(
    uint8_t id,
    const rs485_config_t* config,
    motor_controller_t* controller,
    rs485_private_t* private_data
);
```

**Solution**: Update templates to match actual API

**Files to Modify**:
- `Lib/mavlink_hal/config/generator/templates/stm32/devices.c.j2`
- `Lib/mavlink_hal/config/generator/templates/stm32/handlers.c.j2`

**Changes Needed**:
1. Allocate storage for controllers and private data
2. Call `*_controller_create()` with correct arguments
3. Store pointers to initialized controllers
4. Implement vtable-based unified access

---

### Phase 2: Create Complete YAML Configuration

**Current**: Only RS485 motors in `h753_rs485_test.yaml`

**Needed**: Full configuration with ALL motors

**File**: `Lib/mavlink_hal/config/examples/h753_full.yaml`

**Motor Inventory from `motor_config.h`**:

| Type | IDs | Count | Hardware |
|------|-----|-------|----------|
| Servo | 1-9 | 4 configured | TIM1, TIM2 (PWM) |
| DC Motor | 10-15 | 2 configured | TIM3, TIM4 (PWM + Encoder) |
| RoboMaster | 20-29 | 1 configured (ID 20) | CAN1 (ID 0x201) |
| RS485 | 30-49 | 2 configured (ID 30-31) | USART1, USART2 |

**Configuration Structure**:
```yaml
devices:
  # Servos (IDs 1-4)
  - id: 1
    type: servo
    hardware: { timer: TIM1, channel: 1 }
    config: { min_angle: -90, max_angle: 90, ... }

  # DC Motors (IDs 10-11)
  - id: 10
    type: dc_motor
    hardware: { timer: TIM3, encoder: true }
    config: { pid: {...}, limits: {...} }

  # RoboMaster (ID 20)
  - id: 20
    type: robomaster_motor
    hardware: { can: CAN1, can_id: 0x201 }
    config: { pid_speed: {...}, pid_angle: {...} }

  # RS485 (IDs 30-31)
  - id: 30
    type: rs485_motor
    hardware: { uart: USART1 }
    config: { device_id: 1, motor_index: 0, ... }
```

---

### Phase 3: Template Modifications

**Goal**: Make generated code use existing controller implementations

#### 3.1 Update `devices.c.j2`

**Add Storage Declarations**:
```c
/* Servo controller storage */
static servo_private_t servo_private_data[MAVLINK_GEN_SERVO_COUNT];
static motor_controller_t servo_controllers_storage[MAVLINK_GEN_SERVO_COUNT];
static motor_controller_t* servo_controllers[MAVLINK_GEN_SERVO_COUNT];

/* DC motor controller storage */
static dc_motor_private_t dc_motor_private_data[MAVLINK_GEN_DC_MOTOR_COUNT];
static motor_controller_t dc_motor_controllers_storage[MAVLINK_GEN_DC_MOTOR_COUNT];
static motor_controller_t* dc_motor_controllers[MAVLINK_GEN_DC_MOTOR_COUNT];

/* RoboMaster controller storage */
static robomaster_private_t robomaster_private_data[MAVLINK_GEN_ROBOMASTER_COUNT];
static motor_controller_t robomaster_controllers_storage[MAVLINK_GEN_ROBOMASTER_COUNT];
static motor_controller_t* robomaster_controllers[MAVLINK_GEN_ROBOMASTER_COUNT];

/* RS485 controller storage */
static rs485_private_t rs485_private_data[MAVLINK_GEN_RS485_MOTOR_COUNT];
static motor_controller_t rs485_controllers_storage[MAVLINK_GEN_RS485_MOTOR_COUNT];
static motor_controller_t* rs485_controllers[MAVLINK_GEN_RS485_MOTOR_COUNT];
```

**Update Initialization Logic**:
```c
/* Initialize servos */
for (int i = 0; i < MAVLINK_GEN_SERVO_COUNT; i++) {
    const mavlink_gen_servo_config_t* cfg = &mavlink_gen_servo_configs[i];

    servo_config_t servo_cfg = {
        .id = cfg->id,
        .min_angle = cfg->min_angle,
        .max_angle = cfg->max_angle,
        // ... map all fields
    };

    error_code_t err = servo_controller_create(
        cfg->id,
        &servo_cfg,
        &servo_controllers_storage[i],
        &servo_private_data[i]
    );

    if (err == ERROR_OK) {
        servo_controllers[i] = &servo_controllers_storage[i];
    } else {
        servo_controllers[i] = NULL;
        result = -1;
    }
}
```

#### 3.2 Update `handlers.c.j2`

**Add MAVLink Message Routing**:
```c
void mavlink_gen_handle_message(mavlink_message_t* msg, mavlink_channel_t chan) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_rc_channels_override_t rc;
            mavlink_msg_rc_channels_override_decode(msg, &rc);

            // Map RC channels to motors (1-8 → motors 1-8)
            for (int i = 0; i < 8; i++) {
                uint16_t pwm = rc.chan[i];
                if (pwm != UINT16_MAX) {
                    motor_command_t cmd = {
                        .motor_id = i + 1,
                        .enable = true,
                        .mode = CONTROL_MODE_DUTY_CYCLE,
                        .target_value = (pwm - 1500) / 500.0f,  // -1.0 to 1.0
                    };
                    mavlink_gen_send_motor_command(cmd.motor_id, &cmd);
                }
            }
            break;
        }

        case MAVLINK_MSG_ID_MOTOR_COMMAND: {
            mavlink_motor_command_t mcmd;
            mavlink_msg_motor_command_decode(msg, &mcmd);

            motor_command_t cmd = {
                .motor_id = mcmd.motor_id,
                .enable = mcmd.enable,
                .mode = mcmd.control_mode,
                .target_value = mcmd.target_value,
            };
            mavlink_gen_send_motor_command(cmd.motor_id, &cmd);
            break;
        }

        // Add PARAM_REQUEST_LIST, PARAM_SET, etc.
    }
}
```

**Add Unified Motor Command Function**:
```c
error_code_t mavlink_gen_send_motor_command(uint8_t motor_id, motor_command_t* cmd) {
    motor_controller_t* controller = mavlink_gen_get_controller(motor_id);
    if (!controller || !controller->vtable || !controller->vtable->set_command) {
        return ERROR_INVALID_PARAMETER;
    }
    return controller->vtable->set_command(controller, cmd);
}

motor_controller_t* mavlink_gen_get_controller(uint8_t motor_id) {
    // Servo: 1-9
    if (motor_id >= 1 && motor_id <= 9) {
        for (int i = 0; i < MAVLINK_GEN_SERVO_COUNT; i++) {
            if (servo_controllers[i] && servo_controllers[i]->id == motor_id) {
                return servo_controllers[i];
            }
        }
    }

    // DC: 10-15
    if (motor_id >= 10 && motor_id <= 15) {
        for (int i = 0; i < MAVLINK_GEN_DC_MOTOR_COUNT; i++) {
            if (dc_motor_controllers[i] && dc_motor_controllers[i]->id == motor_id) {
                return dc_motor_controllers[i];
            }
        }
    }

    // RoboMaster: 20-29
    if (motor_id >= 20 && motor_id <= 29) {
        for (int i = 0; i < MAVLINK_GEN_ROBOMASTER_COUNT; i++) {
            if (robomaster_controllers[i] && robomaster_controllers[i]->id == motor_id) {
                return robomaster_controllers[i];
            }
        }
    }

    // RS485: 30-49
    if (motor_id >= 30 && motor_id <= 49) {
        for (int i = 0; i < MAVLINK_GEN_RS485_MOTOR_COUNT; i++) {
            if (rs485_controllers[i] && rs485_controllers[i]->id == motor_id) {
                return rs485_controllers[i];
            }
        }
    }

    return NULL;
}
```

---

### Phase 4: CMakeLists.txt Integration

**Add Generated Files to Build**:

```cmake
# Generated MAVLink HAL code
set(MAVLINK_HAL_GENERATED_DIR ${CMAKE_SOURCE_DIR}/Lib/mavlink_hal/config/generated/h753_full)

set(MAVLINK_HAL_SOURCES
    ${MAVLINK_HAL_GENERATED_DIR}/mavlink_generated_devices.c
    ${MAVLINK_HAL_GENERATED_DIR}/mavlink_generated_handlers.c
)

# Include generated headers
include_directories(${MAVLINK_HAL_GENERATED_DIR})

# Add to target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MAVLINK_HAL_SOURCES}
)
```

**Remove Old motor_registry from Build**:
```cmake
# REMOVE THIS:
# App/motors/motor_registry.c
```

---

### Phase 5: Refactor freertos.c

**Before** (Current):
```c
#include "../../App/motors/motor_registry.h"

// In StartDefaultTask():
motor_registry_init();
motor_registry_create_all_motors();

// In loop:
motor_registry_update_all(delta_time);

// MAVLink handling:
motor_registry_send_command(motor_id, &cmd);
```

**After** (Generated):
```c
#include "mavlink_generated_config.h"
#include "mavlink_generated_devices.h"
#include "mavlink_generated_handlers.h"

// In StartDefaultTask():
int init_result = mavlink_gen_init_all_devices();
if (init_result != 0) {
    // Handle error
}

// In loop:
mavlink_gen_update_all_devices();

// MAVLink handling:
mavlink_message_t msg;
if (mavlink_receive_message(&msg)) {
    mavlink_gen_handle_message(&msg, MAVLINK_COMM_0);
}
```

---

### Phase 6: File Cleanup

**Files to REMOVE**:
```
App/motors/motor_registry.c      (334 lines - replaced by generated)
App/motors/motor_registry.h      (replaced by generated)
App/config/motor_config.h        (490 lines - replaced by YAML)
```

**Files to KEEP** (implementations):
```
App/motors/motor_interface.h     ✅ Core interface
App/motors/servo_controller.c/h  ✅ Servo implementation
App/motors/dc_controller.c/h     ✅ DC motor implementation
App/motors/robomaster_controller.c/h ✅ RoboMaster implementation
App/motors/rs485_controller.c/h  ✅ RS485 implementation
App/hal/hardware_manager.c/h     ✅ Hardware abstraction
App/comm/mavlink_udp.c/h         ✅ MAVLink transport
App/config/parameter_manager.c/h ✅ PID parameters
```

---

## Migration Steps (Detailed)

### Task 1: Fix Code Generator Templates ⚙️

**Files**:
- `Lib/mavlink_hal/config/generator/templates/stm32/devices.c.j2`
- `Lib/mavlink_hal/config/generator/templates/stm32/handlers.c.j2`

**Actions**:
1. Add storage allocation for controllers and private data
2. Update initialization to call `*_controller_create(id, cfg, ctrl, priv)`
3. Add vtable-based unified motor access functions
4. Implement MAVLink message handlers (RC_CHANNELS, MOTOR_COMMAND, PARAM_*)

### Task 2: Create Complete YAML Configuration 📝

**File**: `Lib/mavlink_hal/config/examples/h753_full.yaml`

**Actions**:
1. Extract all motor configs from `App/config/motor_config.h`
2. Convert to YAML format
3. Include all 4 motor types (servo, dc, robomaster, rs485)
4. Validate with `python generator/cli.py validate h753_full.yaml`

### Task 3: Generate Code with Fixed Templates 🔧

**Actions**:
1. Run: `python generator/cli.py generate h753_full.yaml --platform stm32 --output generated/h753_full --force`
2. Verify generated files compile
3. Check API matches h753 controllers

### Task 4: Update CMakeLists.txt 🔨

**Actions**:
1. Add `generated/h753_full/` to include directories
2. Add generated `.c` files to build
3. Remove `motor_registry.c` from sources
4. Test build: `cmake --build build/`

### Task 5: Refactor freertos.c 🔄

**Actions**:
1. Replace `motor_registry` includes with `mavlink_generated_*` includes
2. Replace `motor_registry_init()` → `mavlink_gen_init_all_devices()`
3. Replace `motor_registry_update_all()` → `mavlink_gen_update_all_devices()`
4. Replace manual MAVLink parsing → `mavlink_gen_handle_message()`
5. Test compilation

### Task 6: Remove Old System Files 🗑️

**Actions**:
1. Delete `App/motors/motor_registry.c`
2. Delete `App/motors/motor_registry.h`
3. Delete `App/config/motor_config.h`
4. Update any remaining references
5. Clean build: `rm -rf build && mkdir build`

### Task 7: Build and Test 🧪

**Actions**:
1. Full rebuild: `cd build && cmake .. && cmake --build .`
2. Flash firmware
3. Test each motor type via MAVLink
4. Verify PID parameter system still works
5. Check motor state feedback

### Task 8: Documentation Update 📚

**Actions**:
1. Update CLAUDE.md with new architecture
2. Document YAML configuration format
3. Create migration guide for future changes
4. Update RS485_3MOTOR_PROTOCOL_UPDATE.md

---

## Risk Mitigation

### Backup Strategy
```bash
# Before migration, create backup branch
git checkout -b backup-pre-mavlink-hal-migration
git commit -am "Backup before mavlink_hal migration"
git checkout feature-mavlinkForEveryone
```

### Rollback Plan
If migration fails:
1. Revert freertos.c changes
2. Restore motor_registry.c/h to CMakeLists.txt
3. Keep generated code for reference
4. Debug templates offline

### Testing Checkpoints
- ✅ After Task 3: Generated code compiles
- ✅ After Task 4: Project builds successfully
- ✅ After Task 5: Firmware boots without errors
- ✅ After Task 7: All motors respond to MAVLink commands

---

## Success Criteria

1. ✅ All motors (servo, DC, RoboMaster, RS485) controllable via MAVLink
2. ✅ Configuration is in YAML, not hardcoded C
3. ✅ Generated code matches actual controller APIs
4. ✅ No motor_registry.c/h - fully replaced by generated code
5. ✅ PID parameter system still functional
6. ✅ Build size comparable or smaller
7. ✅ Motor response latency unchanged
8. ✅ Documentation updated

---

## Timeline Estimate

| Task | Estimated Time | Complexity |
|------|----------------|------------|
| 1. Fix Templates | 2-3 hours | High |
| 2. Create YAML | 1 hour | Medium |
| 3. Generate Code | 30 min | Low |
| 4. Update CMake | 30 min | Low |
| 5. Refactor freertos.c | 1 hour | Medium |
| 6. Remove Old Files | 15 min | Low |
| 7. Build & Test | 2 hours | Medium |
| 8. Documentation | 1 hour | Low |
| **Total** | **8-9 hours** | **Medium-High** |

---

## Next Steps

**Immediate Action**: Start with Task 1 - Fix code generator templates

**Decision Point**: After Task 3, verify generated code quality before proceeding with integration

**Final Review**: Before Task 6 (file removal), ensure all functionality is working

---

## Questions to Resolve

1. **Parameter Manager Integration**: How to integrate `parameter_manager.c` with generated params?
2. **MAVLink UDP**: Keep existing `mavlink_udp.c` or generate transport layer too?
3. **Hardware Manager**: Generated code should use existing `hardware_manager.c`, confirm?
4. **Error Handling**: Generated code should follow existing `error_code_t` pattern?

---

**Status**: Ready to proceed with Task 1
**Date**: 2025-11-14
**Author**: Migration Plan for H753 → MAVLink HAL
