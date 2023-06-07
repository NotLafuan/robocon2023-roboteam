#define PNEUM_PIN 16
// Pin configuration for TB6600
#define STEP_PIN 13     // Step pin
#define DIR_PIN_STEP 12 // Direction pin
#define MM_PER_STEP 0.3675
// Home sequence configuration
#define HOME_SPEED 100    // Speed for homing movement (steps per second)
#define HOME_DIRECTION -1 // Direction for homing movement (-1 = counterclockwise)
// Pin configuration for Cytron MD13
#define DIR_PIN_MD 26 // Direction pin
#define PWM_PIN 25    // PWM pin
#define MAX_SPEED 50
#define FEED_MIN_LIMIT 32 // Minimum limit switch pin
#define FEED_MAX_LIMIT 33 // Maximum limit switch pin