/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <EEP564-Final_SCD_inferencing.h>
#include <Arduino_OV767X.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_ov767x/

#include <stdint.h>
#include <stdlib.h>
#include <ArduinoBLE.h> 


/* Constant variables ------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS     160
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS     120

#define GAME_SERVICE_UUID     "12345678-1234-1234-1234-123456789012"
#define GAME_DATA_CHAR_UUID   "87654321-4321-4321-4321-210987654321"
#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

// Constants for hand gestures - ADDED FOR GAME
#define ROCK 0
#define PAPER 1
#define SCISSORS 2

// Hand gesture names for display - ADDED FOR GAME
const char* hand_names[] = {"Rock", "Paper", "Scissors"};

// Game statistics - ADDED FOR GAME
static int games_played = 0;
static int user_wins = 0;
static int system_wins = 0;
static int draws = 0;
static bool game_mode = false;


// BLE Service
BLEService       gameService(GAME_SERVICE_UUID);
BLECharacteristic gameDataChar(
  GAME_DATA_CHAR_UUID,
  BLERead   | BLENotify,
  10               // exactly 10 bytes per notification
);

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Edge Impulse ------------------------------------------------------------- */
class OV7675 : public OV767X {
    public:
        int begin(int resolution, int format, int fps);
        void readFrame(void* buffer);

    private:
        int vsyncPin;
        int hrefPin;
        int pclkPin;
        int xclkPin;

        volatile uint32_t* vsyncPort;
        uint32_t vsyncMask;
        volatile uint32_t* hrefPort;
        uint32_t hrefMask;
        volatile uint32_t* pclkPort;
        uint32_t pclkMask;

        uint16_t width;
        uint16_t height;
        uint8_t bytes_per_pixel;
        uint16_t bytes_per_row;
        uint8_t buf_rows;
        uint16_t buf_size;
        uint8_t resize_height;
        uint8_t *raw_buf;
        void *buf_mem;
        uint8_t *intrp_buf;
        uint8_t *buf_limit;

        void readBuf();
        int allocate_scratch_buffs();
        int deallocate_scratch_buffs();
};

typedef struct {
	size_t width;
	size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return Serial.read();
}

/* Private variables ------------------------------------------------------- */
static OV7675 Cam;
static bool is_initialised = false;

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;
uint32_t resize_col_sz;
uint32_t resize_row_sz;
bool do_resize = false;
bool do_crop = false;

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize);
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);

/* Game Logic Functions - ALL ADDED FOR GAME ----------------------------------------------------- */

// Function to generate system hand
int generateSystemHand() {
    return random(3); // Returns 0, 1, or 2
}

// Function to generate random rule configuration
int generateRandomRule() {
    return random(6); // Returns 0-5 for different rule configurations
}

// Function to determine winner based on hands and rule
int determineWinner(int userHand, int systemHand, int ruleConfig) {
    // Return 1 if user wins, -1 if system wins, 0 if draw
    
    switch (ruleConfig) {
        case 0: // Normal rules: Rock beats Scissors, Scissors beats Paper, Paper beats Rock
            if (userHand == systemHand) return 0; // Draw
            if ((userHand == ROCK && systemHand == SCISSORS) ||
                (userHand == SCISSORS && systemHand == PAPER) ||
                (userHand == PAPER && systemHand == ROCK)) {
                return 1; // User wins
            }
            return -1; // System wins
            
        case 1: // Rock beats Paper, Paper beats Scissors, Scissors beats Rock
            if (userHand == systemHand) return 0; // Draw
            if ((userHand == ROCK && systemHand == PAPER) ||
                (userHand == PAPER && systemHand == SCISSORS) ||
                (userHand == SCISSORS && systemHand == ROCK)) {
                return 1; // User wins
            }
            return -1; // System wins
            
        case 2: // Rock beats Paper, Paper beats Rock, Scissors beats Paper
            if (userHand == systemHand) return 0; // Draw
            if ((userHand == ROCK && systemHand == PAPER) ||
                (userHand == PAPER && systemHand == ROCK) ||
                (userHand == SCISSORS && systemHand == PAPER)) {
                return 1; // User wins
            }
            return -1; // System wins
            
        case 3: // Rock beats Scissors, Scissors beats Rock, Paper beats Scissors
            if (userHand == systemHand) return 0; // Draw
            if ((userHand == ROCK && systemHand == SCISSORS) ||
                (userHand == SCISSORS && systemHand == ROCK) ||
                (userHand == PAPER && systemHand == SCISSORS)) {
                return 1; // User wins
            }
            return -1; // System wins
            
        case 4: // Rock beats Rock, Paper beats Paper, Scissors beats Scissors
            if (userHand == systemHand) return 1; // User wins if same hand
            return -1; // System wins otherwise
            
        case 5: // Rock beats Rock, Paper beats Scissors, Scissors beats Paper
            if (userHand == ROCK && systemHand == ROCK) return 1; // User wins
            if (userHand == PAPER && systemHand == SCISSORS) return 1; // User wins
            if (userHand == SCISSORS && systemHand == PAPER) return 1; // User wins
            if (userHand == systemHand && userHand != ROCK) return 0; // Draw for same hand except Rock
            return -1; // System wins otherwise
            
        default:
            return 0; // Default to draw if something goes wrong
    }
}

// Function to display rule configuration
void displayRuleConfig(int ruleConfig) {
    ei_printf("\n[Game Rule for This Round]\n");
    
    switch (ruleConfig) {
        case 0:
            ei_printf("Rock beats Scissors\n");
            ei_printf("Scissors beats Paper\n");
            ei_printf("Paper beats Rock\n");
            break;
        case 1:
            ei_printf("Rock beats Paper\n");
            ei_printf("Paper beats Scissors\n");
            ei_printf("Scissors beats Rock\n");
            break;
        case 2:
            ei_printf("Rock beats Paper\n");
            ei_printf("Paper beats Rock\n");
            ei_printf("Scissors beats Paper\n");
            break;
        case 3:
            ei_printf("Rock beats Scissors\n");
            ei_printf("Scissors beats Rock\n");
            ei_printf("Paper beats Scissors\n");
            break;
        case 4:
            ei_printf("Rock beats Rock\n");
            ei_printf("Paper beats Paper\n");
            ei_printf("Scissors beats Scissors\n");
            break;
        case 5:
            ei_printf("Rock beats Rock\n");
            ei_printf("Paper beats Scissors\n");
            ei_printf("Scissors beats Paper\n");
            break;
        default:
            ei_printf("Unknown rule configuration\n");
            break;
    }
}

// Function to display game result
void displayGameResult(int result, int userHand, int systemHand) {
    ei_printf("\n=== Game Result ===\n");
    ei_printf("You played: %s\n", hand_names[userHand]);
    ei_printf("System played: %s\n", hand_names[systemHand]);
    
    if (result == 0) {
        ei_printf("Result: It's a DRAW!\n");
        draws++;
    } else if (result == 1) {
        ei_printf("Result: YOU WIN!\n");
        user_wins++;
    } else {
        ei_printf("Result: System Wins\n");
        system_wins++;
    }
    games_played++;
    
    // Show statistics
    ei_printf("\nGames played: %d\n", games_played);
    ei_printf("Your wins: %d (%.1f%%)\n", user_wins, games_played > 0 ? (float)user_wins/games_played*100 : 0);
    ei_printf("System wins: %d (%.1f%%)\n", system_wins, games_played > 0 ? (float)system_wins/games_played*100 : 0);
    ei_printf("Draws: %d (%.1f%%)\n", draws, games_played > 0 ? (float)draws/games_played*100 : 0);
}

void testGameLogic() {
    ei_printf("\n=== Testing Game Logic ===\n");
    
    // Test all rule configurations
    for (int rule = 0; rule < 6; rule++) {
        ei_printf("\nRule %d:\n", rule);
        displayRuleConfig(rule);
        
        // Test a few combinations
        int result1 = determineWinner(ROCK, SCISSORS, rule);
        int result2 = determineWinner(PAPER, ROCK, rule);
        int result3 = determineWinner(SCISSORS, PAPER, rule);
        
        ei_printf("  Rock vs Scissors: %s\n", 
                  result1 == 1 ? "User wins" : result1 == -1 ? "System wins" : "Draw");
        ei_printf("  Paper vs Rock: %s\n", 
                  result2 == 1 ? "User wins" : result2 == -1 ? "System wins" : "Draw");
        ei_printf("  Scissors vs Paper: %s\n", 
                  result3 == 1 ? "User wins" : result3 == -1 ? "System wins" : "Draw");
    }
    
    ei_printf("\n=== Game Logic Test Complete ===\n");
}



/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

   // —— BEGIN BLE SETUP ——————————————————————————————
    if (!BLE.begin()) {
        Serial.println("Failed to start BLE!");
        while (1);
    }

    // Add our service and characteristic
    gameService.addCharacteristic(gameDataChar);
    BLE.addService(gameService);

    // Set an initial empty value (all zeros) for the 10‐byte characteristic
    uint8_t initPayload[10] = {0};
    gameDataChar.writeValue(initPayload, 10);

    // Give the peripheral a name so your Python scanner can find it
    BLE.setLocalName("RPS-Game");  
    BLE.advertise();

    Serial.println("BLE peripheral started, advertising as “RPS-Game”");


    
    // ENHANCED GAME SETUP WITH DEBUG

    ei_printf("Commands:\n");
    ei_printf("  'g' or 'G' - Play a game round\n");
    ei_printf("  'b' or 'B' - Stop inferencing\n");
    ei_printf("  't' or 'T' - Test game logic\n");
    ei_printf("=====================================\n");
    
    // Initialize random seed
    randomSeed(analogRead(0));
    
    // Test game functions
    ei_printf("Testing game functions...\n");
    int testRule = generateRandomRule();
    int testHand = generateSystemHand();
    ei_printf("Random rule generated: %d\n", testRule);
    ei_printf("Random hand generated: %d (%s)\n", testHand, hand_names[testHand]);
    ei_printf("Game functions working!\n\n");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
    
    // Print class labels for debugging
    ei_printf("Model classes: ");
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("%s", ei_classifier_inferencing_categories[i]);
        if (i < EI_CLASSIFIER_LABEL_COUNT - 1) ei_printf(", ");
    }
    ei_printf("\n");
    
    

    // Verify hand names array
    ei_printf("Hand names: ");
    for (int i = 0; i < 3; i++) {
        ei_printf("%s", hand_names[i]);
        if (i < 2) ei_printf(", ");
    }
    ei_printf("\n\nSystem ready! Press 'g' to start playing!\n");
    ei_printf("Current game stats - Played: %d, Wins: %d, Losses: %d, Draws: %d\n\n", 
              games_played, user_wins, system_wins, draws);
}


/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/

void loop()
{
    bool stop_inferencing = false;
    static int ruleConfig = 0;           // Store rule config for the current game
    static int systemHand = 0;           // Store system hand for the current game
    static bool game_params_set = false; // Flag to track if game parameters are set

    while (stop_inferencing == false) {

        // Debug: Show current state
        ei_printf("Loop iteration - Game mode: %s, Params set: %s\n",
                  game_mode ? "ON" : "OFF",
                  game_params_set ? "YES" : "NO");

        // Check for user input first
        if (ei_get_serial_available() > 0) {
            char cmd = ei_get_serial_byte();
            ei_printf("Received command: '%c'\n", cmd);

            if (cmd == 'b' || cmd == 'B') {
                ei_printf("Inferencing stopped by user\r\n");
                stop_inferencing = true;
                continue;
            }
            else if (cmd == 'g' || cmd == 'G') {
                ei_printf("Game mode activated!\n");
                game_mode = true;
                game_params_set = false; // Reset parameters for new game

                // Clear any remaining serial input
                while (ei_get_serial_available() > 0) {
                    ei_get_serial_byte();
                }
            }
            else if (cmd == 't' || cmd == 'T') {
                testGameLogic();
                continue; // Skip the rest of this loop iteration
            }
        }

        // Generate game parameters only once per game
        if (game_mode && !game_params_set) {
            ei_printf("\n--- Starting Rock-Paper-Scissors Game Round ---\n");
            ruleConfig = generateRandomRule();
            systemHand = generateSystemHand();
            game_params_set = true;

            ei_printf("Debug: Generated rule %d, system hand %d (%s)\n",
                      ruleConfig, systemHand, hand_names[systemHand]);

            displayRuleConfig(ruleConfig);
            ei_printf("\nShow your hand gesture in 3 seconds...\n");
            ei_printf("Get ready: 3...\n");
            ei_sleep(1000);
            ei_printf("2...\n");
            ei_sleep(1000);
            ei_printf("1...\n");
            ei_sleep(1000);
            ei_printf("Show your hand NOW!\n");
        }
        else if (!game_mode) {
            ei_printf("\nStarting inferencing in 2 seconds...\n");
            ei_printf("Press 'g' to play Rock-Paper-Scissors, 't' to test, or 'b' to stop\n");

            // Normal delay for regular inference
            if (ei_sleep(2000) != EI_IMPULSE_OK) {
                break;
            }
        }

        ei_printf("Taking photo...\n");

        if (ei_camera_init() == false) {
            ei_printf("ERR: Failed to initialize image sensor\r\n");
            break;
        }

        // choose resize dimensions
        uint32_t resize_col_sz;
        uint32_t resize_row_sz;
        bool do_resize = false;
        int res = calculate_resize_dimensions(
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT,
            &resize_col_sz,
            &resize_row_sz,
            &do_resize
        );
        if (res) {
            ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
            break;
        }

        void *snapshot_mem = NULL;
        uint8_t *snapshot_buf = NULL;
        snapshot_mem = ei_malloc(resize_col_sz * resize_row_sz * 2);
        if (snapshot_mem == NULL) {
            ei_printf("failed to create snapshot_mem\r\n");
            break;
        }
        snapshot_buf = (uint8_t *)DWORD_ALIGN_PTR((uintptr_t)snapshot_mem);

        if (
            ei_camera_capture(
                EI_CLASSIFIER_INPUT_WIDTH,
                EI_CLASSIFIER_INPUT_HEIGHT,
                snapshot_buf
            ) == false
        ) {
            ei_printf("Failed to capture image\r\n");
            if (snapshot_mem) ei_free(snapshot_mem);
            break;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_cutout_get_data;

        //
        //  2) Run classifier exactly ONCE per loop iteration:
        //
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, debug_nn);
        if (ei_error != EI_IMPULSE_OK) {
            ei_printf("Failed to run impulse (%d)\n", ei_error);
            ei_free(snapshot_mem);
            break;
        }

        //
        //  3) Find highest-confidence class → userHand
        //
        int userHand = 0;
        float maxConfidence = result.classification[0].value;
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > maxConfidence) {
                maxConfidence = result.classification[i].value;
                userHand = i;
            }
        }

        //
        //  4) If in game mode and params set, compute game result and BLE-notify:
        //
        if (game_mode && game_params_set) {
            // (a) Validate userHand index, threshold, etc.
            int gameResult = determineWinner(userHand, systemHand, ruleConfig);
            displayGameResult(gameResult, userHand, systemHand);

            // ——— Insert BLE notification block here ———
            {
                // Convert raw confidences → uint16 (×1000)
                uint16_t confRock     = (uint16_t)(result.classification[0].value * 1000.0f);
                uint16_t confPaper    = (uint16_t)(result.classification[1].value * 1000.0f);
                uint16_t confScissors = (uint16_t)(result.classification[2].value * 1000.0f);

                // Map gameResult (–1/0/1) → resultCode (0/1/2)
                uint8_t resultCode;
                if      (gameResult ==  0) resultCode = 0;  // Draw
                else if (gameResult ==  1) resultCode = 1;  // User wins
                else                        resultCode = 2;  // System wins

                // Pack into 10 bytes (little-endian)
                uint8_t payload[10];
                payload[0] = (uint8_t)ruleConfig;
                payload[1] = (uint8_t)systemHand;
                payload[2] = (uint8_t)userHand;
                payload[3] = resultCode;

                payload[4] = (uint8_t)(confRock & 0xFF);
                payload[5] = (uint8_t)((confRock >> 8) & 0xFF);

                payload[6] = (uint8_t)(confPaper & 0xFF);
                payload[7] = (uint8_t)((confPaper >> 8) & 0xFF);

                payload[8] = (uint8_t)(confScissors & 0xFF);
                payload[9] = (uint8_t)((confScissors >> 8) & 0xFF);

                // Send over BLE if connected
                if (BLE.connected()) {
                    gameDataChar.writeValue(payload, 10);
                    ei_printf(
                      "► Sent BLE notification: [%d %d %d %d | %u %u %u]\n",
                      payload[0], payload[1], payload[2], payload[3],
                      confRock, confPaper, confScissors
                    );
                }
                else {
                    ei_printf("⚠ BLE not connected; skipped notification\n");
                }
            }
            // ——————————————————————————————————————————————

            // 5) Reset game mode
            game_mode = false;
            game_params_set = false;
            ei_printf(
              "\n=== Press 'g' to play again, 't' to test logic, or 'b' to stop ===\n"
            );
        }

        //
        //  6) Print the rest of your inference results exactly as before:
        //
        ei_printf(
          "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
          result.timing.dsp,
          result.timing.classification,
          result.timing.anomaly
        );

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
        ei_printf("Object detection bounding boxes:\r\n");
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) {
                continue;
            }
            ei_printf(
              "  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
              bb.label,
              bb.value,
              bb.x,
              bb.y,
              bb.width,
              bb.height
            );
        }
#else
        // Print classification results
        ei_printf("Classification Results:\r\n");

        // Find highest confidence prediction again (for printing)
        int userHand2 = 0;
        float maxConfidence2 = result.classification[0].value;

        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            ei_printf(
              "  %s: %.5f",
              ei_classifier_inferencing_categories[i],
              result.classification[i].value
            );

            if (game_mode) {
                ei_printf(" (%.1f%%)", result.classification[i].value * 100);
            }

            // Find the class with highest confidence
            if (result.classification[i].value > maxConfidence2) {
                maxConfidence2 = result.classification[i].value;
                userHand2 = i;
            }
            ei_printf("\r\n");
        }

        ei_printf(
          "Debug: Highest confidence class index: %d, confidence: %.3f\n",
          userHand2,
          maxConfidence2
        );

        // Process game result if in game mode (this block only prints, since we already handled BLE above)
        if (game_mode && game_params_set) {
            ei_printf("\n=== GAME PROCESSING ===\n");

            // Validate userHand index
            if (userHand2 >= 0 && userHand2 < 3) {
                ei_printf(
                  "Detected gesture: %s (%.1f%% confidence)\n",
                  hand_names[userHand2],
                  maxConfidence2 * 100
                );
            } else {
                ei_printf(
                  "Error: Invalid hand gesture index %d\n",
                  userHand2
                );
                userHand2 = 0; // Default to Rock if invalid
            }

            // Check confidence threshold
            if (maxConfidence2 < 0.3) {
                ei_printf(
                  "Confidence too low (%.1f%%)! Please try again.\n",
                  maxConfidence2 * 100
                );
                ei_printf(
                  "Make sure your hand gesture is clear and well-lit.\n"
                );
                ei_printf("Minimum confidence required: 30%%\n");
            } else {
                if (maxConfidence2 < 0.6) {
                    ei_printf(
                      "Warning: Low confidence (%.1f%%). Result may be inaccurate.\n",
                      maxConfidence2 * 100
                    );
                }

                ei_printf(
                  "Debug: Calling determineWinner(%d, %d, %d)\n",
                  userHand2, systemHand, ruleConfig
                );
                int gameResult2 =
                  determineWinner(userHand2, systemHand, ruleConfig);
                ei_printf("Debug: Game result = %d\n", gameResult2);

                displayGameResult(gameResult2, userHand2, systemHand);
            }

            // Reset game mode and parameters
            game_mode = false;
            game_params_set = false;
            ei_printf(
              "\n=== Press 'g' to play again, 't' to test logic, or 'b' to stop ===\n"
            );
        }
#endif

        // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
        ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
        ei_printf("Visual anomalies:\r\n");
        for (uint32_t i = 0; i < result.visual_ad_count; i++) {
            ei_impulse_result_bounding_box_t bb =
              result.visual_ad_grid_cells[i];
            if (bb.value == 0) {
                continue;
            }
            ei_printf(
              "  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
              bb.label,
              bb.value,
              bb.x,
              bb.y,
              bb.width,
              bb.height
            );
        }
#endif

        // Clean up memory
        if (snapshot_mem) {
            ei_free(snapshot_mem);
            snapshot_mem = NULL;
        }

        ei_printf("End of loop iteration\n\n");
    } // end while

    ei_printf("Exiting main loop, cleaning up camera\n");
    ei_camera_deinit();
}


/**
 * @brief      Determine whether to resize and to which dimension
 *
 * @param[in]  out_width     width of output image
 * @param[in]  out_height    height of output image
 * @param[out] resize_col_sz       pointer to frame buffer's column/width value
 * @param[out] resize_row_sz       pointer to frame buffer's rows/height value
 * @param[out] do_resize     returns whether to resize (or not)
 *
 */
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize)
{
    size_t list_size = 2;
    const ei_device_resize_resolutions_t list[list_size] = { {42,32}, {128,96} };

    // (default) conditions
    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }

    return 0;
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    if (!Cam.begin(QQVGA, RGB565, 1)) { // VGA downsampled to QQVGA (OV7675)
        ei_printf("ERR: Failed to initialize camera\r\n");
        return false;
    }
    is_initialised = true;

    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    if (is_initialised) {
        Cam.end();
        is_initialised = false;
    }
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           when full resolution is expected.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    if (!out_buf) {
        ei_printf("ERR: invalid parameters\r\n");
        return false;
    }

    // choose resize dimensions
    int res = calculate_resize_dimensions(img_width, img_height, &resize_col_sz, &resize_row_sz, &do_resize);
    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        return false;
    }

    if ((img_width != resize_col_sz)
        || (img_height != resize_row_sz)) {
        do_crop = true;
    }

    Cam.readFrame(out_buf); // captures image and resizes

    if (do_crop) {
        uint32_t crop_col_sz;
        uint32_t crop_row_sz;
        uint32_t crop_col_start;
        uint32_t crop_row_start;
        crop_row_start = (resize_row_sz - img_height) / 2;
        crop_col_start = (resize_col_sz - img_width) / 2;
        crop_col_sz = img_width;
        crop_row_sz = img_height;

        //ei_printf("crop cols: %d, rows: %d\r\n", crop_col_sz,crop_row_sz);
        cropImage(resize_col_sz, resize_row_sz,
                out_buf,
                crop_col_start, crop_row_start,
                crop_col_sz, crop_row_sz,
                out_buf,
                16);
    }

    // The following variables should always be assigned
    // if this routine is to return true
    // cutout values
    //ei_camera_snapshot_is_resized = do_resize;
    //ei_camera_snapshot_is_cropped = do_crop;
    ei_camera_capture_out = out_buf;

    return true;
}

/**
 * @brief      Convert RGB565 raw camera buffer to RGB888
 *
 * @param[in]   offset       pixel offset of raw buffer
 * @param[in]   length       number of pixels to convert
 * @param[out]  out_buf      pointer to store output image
 */
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 2;
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {
        // grab the value and convert to r/g/b
        uint16_t pixel = (ei_camera_capture_out[pixel_ix] << 8) | ei_camera_capture_out[pixel_ix+1];
        uint8_t r, g, b;
        r = ((pixel >> 11) & 0x1f) << 3;
        g = ((pixel >> 5) & 0x3f) << 2;
        b = (pixel & 0x1f) << 3;

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        pixel_ix+=2;
        bytes_left--;
    }

    // and done!
    return 0;
}

// This include file works in the Arduino environment
// to define the Cortex-M intrinsics
#ifdef __ARM_FEATURE_SIMD32
#include <device.h>
#endif
// This needs to be < 16 or it won't fit. Cortex-M4 only has SIMD for signed multiplies
#define FRAC_BITS 14
#define FRAC_VAL (1<<FRAC_BITS)
#define FRAC_MASK (FRAC_VAL - 1)
//
// Resize
//
// Assumes that the destination buffer is dword-aligned
// Can be used to resize the image smaller or larger
// If resizing much smaller than 1/3 size, then a more rubust algorithm should average all of the pixels
// This algorithm uses bilinear interpolation - averages a 2x2 region to generate each new pixel
//
// Optimized for 32-bit MCUs
// supports 8 and 16-bit pixels
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
    uint32_t src_x_accum, src_y_accum; // accumulators and fractions for scaling the image
    uint32_t x_frac, nx_frac, y_frac, ny_frac;
    int x, y, ty, tx;

    if (iBpp != 8 && iBpp != 16)
        return;
    src_y_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
    const uint32_t src_x_frac = (srcWidth * FRAC_VAL) / dstWidth;
    const uint32_t src_y_frac = (srcHeight * FRAC_VAL) / dstHeight;
    const uint32_t r_mask = 0xf800f800;
    const uint32_t g_mask = 0x07e007e0;
    const uint32_t b_mask = 0x001f001f;
    uint8_t *s, *d;
    uint16_t *s16, *d16;
    uint32_t x_frac2, y_frac2; // for 16-bit SIMD
    for (y=0; y < dstHeight; y++) {
        ty = src_y_accum >> FRAC_BITS; // src y
        y_frac = src_y_accum & FRAC_MASK;
        src_y_accum += src_y_frac;
        ny_frac = FRAC_VAL - y_frac; // y fraction and 1.0 - y fraction
        y_frac2 = ny_frac | (y_frac << 16); // for M4/M4 SIMD
        s = &srcImage[ty * srcWidth];
        s16 = (uint16_t *)&srcImage[ty * srcWidth * 2];
        d = &dstImage[y * dstWidth];
        d16 = (uint16_t *)&dstImage[y * dstWidth * 2];
        src_x_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
        if (iBpp == 8) {
        for (x=0; x < dstWidth; x++) {
            uint32_t tx, p00,p01,p10,p11;
            tx = src_x_accum >> FRAC_BITS;
            x_frac = src_x_accum & FRAC_MASK;
            nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
            x_frac2 = nx_frac | (x_frac << 16);
            src_x_accum += src_x_frac;
            p00 = s[tx]; p10 = s[tx+1];
            p01 = s[tx+srcWidth]; p11 = s[tx+srcWidth+1];
    #ifdef __ARM_FEATURE_SIMD32
            p00 = __SMLAD(p00 | (p10<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            p01 = __SMLAD(p01 | (p11<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            p00 = __SMLAD(p00 | (p01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
    #else // generic C code
            p00 = ((p00 * nx_frac) + (p10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            p01 = ((p01 * nx_frac) + (p11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            p00 = ((p00 * ny_frac) + (p01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
    #endif // Cortex-M4/M7
            *d++ = (uint8_t)p00; // store new pixel
        } // for x
        } // 8-bpp
        else
        { // RGB565
        for (x=0; x < dstWidth; x++) {
            uint32_t tx, p00,p01,p10,p11;
            uint32_t r00, r01, r10, r11, g00, g01, g10, g11, b00, b01, b10, b11;
            tx = src_x_accum >> FRAC_BITS;
            x_frac = src_x_accum & FRAC_MASK;
            nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
            x_frac2 = nx_frac | (x_frac << 16);
            src_x_accum += src_x_frac;
            p00 = __builtin_bswap16(s16[tx]); p10 = __builtin_bswap16(s16[tx+1]);
            p01 = __builtin_bswap16(s16[tx+srcWidth]); p11 = __builtin_bswap16(s16[tx+srcWidth+1]);
    #ifdef __ARM_FEATURE_SIMD32
            {
            p00 |= (p10 << 16);
            p01 |= (p11 << 16);
            r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
            r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
            r00 = __SMLAD(r00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            r01 = __SMLAD(r01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            r00 = __SMLAD(r00 | (r01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            g00 = __SMLAD(g00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            g01 = __SMLAD(g01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            g00 = __SMLAD(g00 | (g01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            b00 = __SMLAD(b00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            b01 = __SMLAD(b01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            b00 = __SMLAD(b00 | (b01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            }
    #else // generic C code
            {
            r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
            r10 = (p10 & r_mask) >> 1; g10 = p10 & g_mask; b10 = p10 & b_mask;
            r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
            r11 = (p11 & r_mask) >> 1; g11 = p11 & g_mask; b11 = p11 & b_mask;
            r00 = ((r00 * nx_frac) + (r10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            r01 = ((r01 * nx_frac) + (r11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            r00 = ((r00 * ny_frac) + (r01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            g00 = ((g00 * nx_frac) + (g10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            g01 = ((g01 * nx_frac) + (g11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            g00 = ((g00 * ny_frac) + (g01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            b00 = ((b00 * nx_frac) + (b10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            b01 = ((b01 * nx_frac) + (b11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            b00 = ((b00 * ny_frac) + (b01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            }
    #endif // Cortex-M4/M7
            r00 = (r00 << 1) & r_mask;
            g00 = g00 & g_mask;
            b00 = b00 & b_mask;
            p00 = (r00 | g00 | b00); // re-combine color components
            *d16++ = (uint16_t)__builtin_bswap16(p00); // store new pixel
        } // for x
        } // 16-bpp
    } // for y
} /* resizeImage() */
//
// Crop
//
// Assumes that the destination buffer is dword-aligned
// optimized for 32-bit MCUs
// Supports 8 and 16-bit pixels
//
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
    uint32_t *s32, *d32;
    int x, y;

    if (startX < 0 || startX >= srcWidth || startY < 0 || startY >= srcHeight || (startX + dstWidth) > srcWidth || (startY + dstHeight) > srcHeight)
       return; // invalid parameters
    if (iBpp != 8 && iBpp != 16)
       return;

    if (iBpp == 8) {
      uint8_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = &srcImage[srcWidth * (y + startY) + startX];
        d = &dstImage[(dstWidth * y)];
        x = 0;
        if ((intptr_t)s & 3 || (intptr_t)d & 3) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it byte-by-byte
          }
        } else {
          // move 4 bytes at a time if aligned or alignment not enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-3; x+= 4) {
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint8_t *)s32;
          d = (uint8_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 8-bpp
    else
    {
      uint16_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = (uint16_t *)&srcImage[2 * srcWidth * (y + startY) + startX * 2];
        d = (uint16_t *)&dstImage[(dstWidth * y * 2)];
        x = 0;
        if ((intptr_t)s & 2 || (intptr_t)d & 2) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it 16-bits at a time
          }
        } else {
          // move 4 bytes at a time if aligned or alignment no enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-1; x+= 2) { // we can move 2 pixels at a time
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint16_t *)s32;
          d = (uint16_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 16-bpp case
} /* cropImage() */

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

// OV767X camera library override
#include <Arduino.h>
#include <Wire.h>

#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 32))
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)

//
// OV7675::begin()
//
// Extends the OV767X library function. Some private variables are needed
// to use the OV7675::readFrame function.
//
int OV7675::begin(int resolution, int format, int fps)
{
    pinMode(OV7670_VSYNC, INPUT);
    pinMode(OV7670_HREF, INPUT);
    pinMode(OV7670_PLK, INPUT);
    pinMode(OV7670_XCLK, OUTPUT);

    vsyncPort = portInputRegister(digitalPinToPort(OV7670_VSYNC));
    vsyncMask = digitalPinToBitMask(OV7670_VSYNC);
    hrefPort = portInputRegister(digitalPinToPort(OV7670_HREF));
    hrefMask = digitalPinToBitMask(OV7670_HREF);
    pclkPort = portInputRegister(digitalPinToPort(OV7670_PLK));
    pclkMask = digitalPinToBitMask(OV7670_PLK);

    // init driver to use full image sensor size
    bool ret = OV767X::begin(VGA, format, fps);
    width = OV767X::width(); // full sensor width
    height = OV767X::height(); // full sensor height
    bytes_per_pixel = OV767X::bytesPerPixel();
    bytes_per_row = width * bytes_per_pixel; // each pixel is 2 bytes
    resize_height = 2;

    buf_mem = NULL;
    raw_buf = NULL;
    intrp_buf = NULL;
    //allocate_scratch_buffs();

    return ret;
} /* OV7675::begin() */

int OV7675::allocate_scratch_buffs()
{
    //ei_printf("allocating buffers..\r\n");
    buf_rows = height / resize_row_sz * resize_height;
    buf_size = bytes_per_row * buf_rows;

    buf_mem = ei_malloc(buf_size);
    if(buf_mem == NULL) {
        ei_printf("failed to create buf_mem\r\n");
        return false;
    }
    raw_buf = (uint8_t *)DWORD_ALIGN_PTR((uintptr_t)buf_mem);

    //ei_printf("allocating buffers OK\r\n");
    return 0;
}

int OV7675::deallocate_scratch_buffs()
{
    //ei_printf("deallocating buffers...\r\n");
    ei_free(buf_mem);
    buf_mem = NULL;

    //ei_printf("deallocating buffers OK\r\n");
    return 0;
}

//
// OV7675::readFrame()
//
// Overrides the OV767X library function. Fixes the camera output to be
// a far more desirable image. This image utilizes the full sensor size
// and has the correct aspect ratio. Since there is limited memory on the
// Nano we bring in only part of the entire sensor at a time and then
// interpolate to a lower resolution.
//
void OV7675::readFrame(void* buffer)
{
    allocate_scratch_buffs();

    uint8_t* out = (uint8_t*)buffer;
    noInterrupts();

    // Falling edge indicates start of frame
    while ((*vsyncPort & vsyncMask) == 0); // wait for HIGH
    while ((*vsyncPort & vsyncMask) != 0); // wait for LOW

    int out_row = 0;
    for (int raw_height = 0; raw_height < height; raw_height += buf_rows) {
        // read in 640xbuf_rows buffer to work with
        readBuf();

        resizeImage(width, buf_rows,
                    raw_buf,
                    resize_col_sz, resize_height,
                    &(out[out_row]),
                    16);

        out_row += resize_col_sz * resize_height * bytes_per_pixel; /* resize_col_sz * 2 * 2 */
    }

    interrupts();

    deallocate_scratch_buffs();
} /* OV7675::readFrame() */

//
// OV7675::readBuf()
//
// Extends the OV767X library function. Reads buf_rows VGA rows from the
// image sensor.
//
void OV7675::readBuf()
{
    int offset = 0;

    uint32_t ulPin = 33; // P1.xx set of GPIO is in 'pin' 32 and above
    NRF_GPIO_Type * port;

    port = nrf_gpio_pin_port_decode(&ulPin);

    for (int i = 0; i < buf_rows; i++) {
        // rising edge indicates start of line
        while ((*hrefPort & hrefMask) == 0); // wait for HIGH

        for (int col = 0; col < bytes_per_row; col++) {
            // rising edges clock each data byte
            while ((*pclkPort & pclkMask) != 0); // wait for LOW

            uint32_t in = port->IN; // read all bits in parallel

            in >>= 2; // place bits 0 and 1 at the "bottom" of the register
            in &= 0x3f03; // isolate the 8 bits we care about
            in |= (in >> 6); // combine the upper 6 and lower 2 bits

            raw_buf[offset++] = in;

            while ((*pclkPort & pclkMask) == 0); // wait for HIGH
        }

        while ((*hrefPort & hrefMask) != 0); // wait for LOW
    }
} /* OV7675::readBuf() */
