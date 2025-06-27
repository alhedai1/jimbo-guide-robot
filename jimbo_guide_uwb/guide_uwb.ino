
// New: 4 UWB anchors
Serial1.begin(115200);   //UWB Anchor 1 (Front-Left)
Serial2.begin(115200);   //UWB Anchor 2 (Front-Right)  
Serial3.begin(115200);   //UWB Anchor 3 (Back-Left)
Serial4.begin(115200);   //UWB Anchor 4 (Back-Right)

// New: 4 anchors, 1 tag
float anchor1_distance = 1.0;  // Front-Left
float anchor2_distance = 1.0;  // Front-Right
float anchor3_distance = 1.0;  // Back-Left
float anchor4_distance = 1.0;  // Back-Right

// Kalman filtered distances
float kalman_anchor1_distance = 1.0;
float kalman_anchor2_distance = 1.0;
float kalman_anchor3_distance = 1.0;
float kalman_anchor4_distance = 1.0;

// 2D Position calculation using 4 anchors
struct Position2D {
    float x, y;
    float confidence;
};

Position2D calculate_2d_position() {
    // Use trilateration with 4 anchors
    // Known anchor positions (calibrated)
    float anchor1_x = 0.0, anchor1_y = 0.0;  // Front-Left
    float anchor2_x = 0.3, anchor2_y = 0.0;  // Front-Right
    float anchor3_x = 0.0, anchor3_y = 0.4;  // Back-Left
    float anchor4_x = 0.3, anchor4_y = 0.4;  // Back-Right
    
    // Calculate tag position using least squares method
    // Return x, y coordinates and confidence
}

// Target distance from person (adjustable)
#define TARGET_DISTANCE 1.2  // meters
#define MIN_SAFE_DISTANCE 0.8
#define MAX_FOLLOW_DISTANCE 2.0

// Calculate optimal following position
float calculate_following_distance() {
    // Use 4-anchor positioning to determine:
    // 1. Current distance from person
    // 2. Relative angle to person
    // 3. Optimal following position
}

// Guide robot specific control modes
enum GuideMode {
    FOLLOW_PERSON,      // Normal following
    MAINTAIN_DISTANCE,  // Keep fixed distance
    EMERGENCY_STOP,     // Too close or lost signal
    SEARCH_PERSON,      // Lost person, search mode
    OBSTACLE_AVOID      // Obstacle detected
};

GuideMode determine_guide_mode() {
    if (person_distance < MIN_SAFE_DISTANCE) {
        return EMERGENCY_STOP;
    } else if (person_distance > MAX_FOLLOW_DISTANCE) {
        return SEARCH_PERSON;
    } else if (obstacle_detected) {
        return OBSTACLE_AVOID;
    } else {
        return MAINTAIN_DISTANCE;
    }
}

void setup_uwb_anchors() {
    Serial1.begin(115200);  // Anchor 1
    Serial2.begin(115200);  // Anchor 2
    Serial3.begin(115200);  // Anchor 3
    Serial4.begin(115200);  // Anchor 4
    
    // Initialize all anchors
    request_sensor_distance_data_4anchor();
}

void request_sensor_distance_data_4anchor() {
    Serial1.print("reset\r");
    Serial2.print("reset\r");
    Serial3.print("reset\r");
    Serial4.print("reset\r");
    delay(1000);
    
    // Enable LEC mode on all anchors
    Serial1.print("lec");
    Serial1.write(0x0d);
    Serial2.print("lec");
    Serial2.write(0x0d);
    Serial3.print("lec");
    Serial3.write(0x0d);
    Serial4.print("lec");
    Serial4.write(0x0d);
}

void get_4anchor_distance_data() {
    String anchor1_string = Serial1.readStringUntil('\n');
    String anchor2_string = Serial2.readStringUntil('\n');
    String anchor3_string = Serial3.readStringUntil('\n');
    String anchor4_string = Serial4.readStringUntil('\n');
    
    // Parse distance data from each anchor
    anchor1_distance = parse_distance(anchor1_string);
    anchor2_distance = parse_distance(anchor2_string);
    anchor3_distance = parse_distance(anchor3_string);
    anchor4_distance = parse_distance(anchor4_string);
}
