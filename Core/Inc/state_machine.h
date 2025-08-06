// 7. state_machine.h - Path execution state machine
/*
 * state_machine.h
 *
 * State machine for micromouse path execution
 * Handles search run, fast run, and diagonal movement states
 *
 * Author: Micromouse Project
 * Date: 2025
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <advanced_algorithms.h>
#include <audio.h>
#include <stdint.h>
#include <stdbool.h>

// Main micromouse states
typedef enum {
    STATE_IDLE,             // Waiting for start command
    STATE_CALIBRATE,        // Sensor calibration
    STATE_SEARCH_RUN,       // Maze exploration phase
    STATE_RETURN_TO_START,  // Return to starting position
    STATE_PREPARE_FAST,     // Prepare for fast run
    STATE_FAST_RUN,         // High-speed run to center
    STATE_COMPLETE,         // Run completed
    STATE_ERROR             // Error state
} MicromouseState;

// Movement execution states
typedef enum {
    MOVE_IDLE,              // No movement
    MOVE_FORWARD,           // Moving forward
    MOVE_TURN_LEFT,         // Turning left
    MOVE_TURN_RIGHT,        // Turning right
    MOVE_TURN_180,          // 180-degree turn
    MOVE_DIAGONAL_NE,       // Northeast diagonal
    MOVE_DIAGONAL_SE,       // Southeast diagonal
    MOVE_DIAGONAL_SW,       // Southwest diagonal
    MOVE_DIAGONAL_NW,       // Northwest diagonal
    MOVE_CURVE_LEFT,        // Curved left turn
    MOVE_CURVE_RIGHT,       // Curved right turn
    MOVE_COMPLETE           // Movement complete
} MovementState;

// Search behavior states
typedef enum {
    SEARCH_EXPLORE,         // Exploring unknown areas
    SEARCH_RETURN,          // Returning to start
    SEARCH_GOAL_REACHED,    // Goal has been reached
    SEARCH_OPTIMIZE         // Optimizing path
} SearchState;

// State machine data structure
typedef struct {
    MicromouseState main_state;         // Main state
    MovementState movement_state;       // Current movement state
    SearchState search_state;           // Search behavior state

    uint32_t state_start_time;          // Time when current state started
    uint32_t state_timeout;             // State timeout value

    bool state_complete;                // Current state completion flag
    bool emergency_stop;                // Emergency stop flag

    // Movement execution data
    MotionCommand current_command;      // Current motion command
    float movement_progress;            // Movement completion percentage

    // Performance tracking
    uint32_t search_start_time;         // Search run start time
    uint32_t fast_run_start_time;       // Fast run start time
    uint16_t search_moves;              // Number of moves in search
    uint16_t fast_moves;                // Number of moves in fast run
} StateMachine;

// Global state machine instance
extern StateMachine mouse_state;

// State machine control functions
void StateMachine_Initialize(void);
void StateMachine_Update(void);
void StateMachine_Start(void);
void StateMachine_Stop(void);
void StateMachine_EmergencyStop(void);
void StateMachine_Reset(void);

// State transition functions
void StateMachine_SetState(MicromouseState new_state);
void StateMachine_SetMovementState(MovementState new_state);
void StateMachine_SetSearchState(SearchState new_state);

// Main state handlers
void StateMachine_HandleIdle(void);
void StateMachine_HandleCalibrate(void);
void StateMachine_HandleSearchRun(void);
void StateMachine_HandleReturnToStart(void);
void StateMachine_HandlePrepareFast(void);
void StateMachine_HandleFastRun(void);
void StateMachine_HandleComplete(void);
void StateMachine_HandleError(void);

// Movement execution functions
void StateMachine_ExecuteMovement(MotionCommand command);
bool StateMachine_IsMovementComplete(void);
void StateMachine_AbortMovement(void);

// Search run specific functions
void StateMachine_ProcessSearchMove(void);
void StateMachine_UpdateMazeFromSensors(void);
bool StateMachine_IsGoalReached(void);
void StateMachine_OptimizeSearchPath(void);

// Fast run specific functions
void StateMachine_ExecuteFastRun(void);
void StateMachine_ExecuteDiagonalSequence(void);
void StateMachine_ExecuteCurvedPath(void);

// Diagonal movement handlers
void StateMachine_ExecuteDiagonalNE(void);
void StateMachine_ExecuteDiagonalSE(void);
void StateMachine_ExecuteDiagonalSW(void);
void StateMachine_ExecuteDiagonalNW(void);

// Curved movement handlers
void StateMachine_ExecuteCurveLeft(void);
void StateMachine_ExecuteCurveRight(void);

// Safety and error handling
bool StateMachine_CheckSafety(void);
void StateMachine_HandleSensorError(void);
void StateMachine_HandleMotorError(void);
void StateMachine_RecoverFromError(void);

// Performance monitoring
void StateMachine_UpdatePerformanceStats(void);
void StateMachine_PrintRunSummary(void);

// Debug functions
void StateMachine_PrintCurrentState(void);
const char* StateMachine_GetStateName(MicromouseState state);
const char* StateMachine_GetMovementName(MovementState state);

#endif // STATE_MACHINE_H
