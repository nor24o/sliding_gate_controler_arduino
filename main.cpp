#include <Bounce2.h>
#include <EEPROM.h>
#include <RCSwitch.h>

// Pin definitions
const int emergencyStopPin = 2;
const int rfPin = 3;
const int closedLimitSwitchPin = 4;
const int openedLimitSwitchPin = 5;
const int pauseMovementPin = 6;
const int changeDirectionAndRunPin = 7;
const int directionRelayPin = 8;
const int mainRelayPin = 9;
const int photoCellBarrierPin = 10;

// RF codes
const unsigned long openCode = 111222333;
const unsigned long closeCode = 112223334;
const unsigned long stopCode = 1122335;

// EEPROM addresses
const int isPausedAddr = 0;
const int isRunningAddr = 1;
const int directionAddr = 2;
const int photoCellTriggeredAddr = 3;
const int checksumAddr = 4;

// Debounce objects
Bounce emergencyStopDebouncer = Bounce();
Bounce closedLimitSwitchDebouncer = Bounce();
Bounce openedLimitSwitchDebouncer = Bounce();
Bounce pauseMovementDebouncer = Bounce();
Bounce changeDirectionAndRunDebouncer = Bounce();
Bounce photoCellBarrierDebouncer = Bounce();

// RF receiver object
RCSwitch rfReceiver = RCSwitch();

// State variables
bool isPaused = false;
bool isRunning = false;
bool direction = false; // false = closing, true = opening
unsigned long lastDirectionChangeTime = 0;
unsigned long lastMotorStartTime = 0;
bool photoCellTriggered = false;

// Timing variables
const unsigned long directionChangeDelay = 5000; // 2 seconds delay between direction changes
const unsigned long motorStartDelay = 2000;       // 0.5 seconds delay to start motor after direction change
const unsigned long detailsPrintInterval = 2000; // 2 seconds interval for printing details
unsigned long lastDetailsPrintTime = 0;

void setup() {
  // Initialize inputs with debounce
  pinMode(emergencyStopPin, INPUT_PULLUP);
  emergencyStopDebouncer.attach(emergencyStopPin);
  emergencyStopDebouncer.interval(25);

  pinMode(closedLimitSwitchPin, INPUT_PULLUP);
  closedLimitSwitchDebouncer.attach(closedLimitSwitchPin);
  closedLimitSwitchDebouncer.interval(25);

  pinMode(openedLimitSwitchPin, INPUT_PULLUP);
  openedLimitSwitchDebouncer.attach(openedLimitSwitchPin);
  openedLimitSwitchDebouncer.interval(25);

  pinMode(pauseMovementPin, INPUT_PULLUP);
  pauseMovementDebouncer.attach(pauseMovementPin);
  pauseMovementDebouncer.interval(25);

  pinMode(changeDirectionAndRunPin, INPUT_PULLUP);
  changeDirectionAndRunDebouncer.attach(changeDirectionAndRunPin);
  changeDirectionAndRunDebouncer.interval(25);

  pinMode(photoCellBarrierPin, INPUT_PULLUP);
  photoCellBarrierDebouncer.attach(photoCellBarrierPin);
  photoCellBarrierDebouncer.interval(25);

  // Initialize outputs
  pinMode(directionRelayPin, OUTPUT);
  pinMode(mainRelayPin, OUTPUT);

  // Initialize RF receiver
  rfReceiver.enableReceive(digitalPinToInterrupt(rfPin));

  // Read saved state from EEPROM
  if (!readStateFromEEPROM()) {
    // If EEPROM data is corrupted, reset to default values
    isPaused = false;
    isRunning = false;
    direction = false; // Default to closing direction
    photoCellTriggered = false;
    saveStateIfChanged();
  }

  // Initialize motor state based on EEPROM values
  if (isRunning) {
    runMotor(direction);
  } else {
    stopMotor();
  }

  Serial.begin(9600);
}

void loop() {
  // Update debouncers
  emergencyStopDebouncer.update();
  closedLimitSwitchDebouncer.update();
  openedLimitSwitchDebouncer.update();
  pauseMovementDebouncer.update();
  changeDirectionAndRunDebouncer.update();
  photoCellBarrierDebouncer.update();

  // Read debounced inputs
  bool emergencyStop = emergencyStopDebouncer.read() == LOW;
  bool closedLimitSwitch = closedLimitSwitchDebouncer.read() == LOW;
  bool openedLimitSwitch = openedLimitSwitchDebouncer.read() == LOW;
  bool pauseMovement = pauseMovementDebouncer.read() == LOW;
  bool changeDirectionAndRun = changeDirectionAndRunDebouncer.read() == LOW;
  bool photoCellBarrier = photoCellBarrierDebouncer.read() == LOW;

  unsigned long currentTime = millis();

  // Handle RF commands
  if (rfReceiver.available()) {
    unsigned long rfCode = rfReceiver.getReceivedValue();
    rfReceiver.resetAvailable();

    if (rfCode == openCode) {
      handleOpenCommand();
    } else if (rfCode == closeCode) {
      handleCloseCommand();
    } else if (rfCode == stopCode) {
      handlePauseMovement(true);
    }
  }

  if (emergencyStop) {
    handleEmergencyStop();
  } else if (photoCellBarrier && !closedLimitSwitch) {
    handlePhotoCellBarrier();
  } else {
    handlePauseMovement(pauseMovement);
    handleChangeDirectionAndRun(changeDirectionAndRun, currentTime);
    handleLimitSwitches(closedLimitSwitch, openedLimitSwitch);
    resumeMotorIfDelayed(currentTime);
  }

  if (currentTime - lastDetailsPrintTime >= detailsPrintInterval) {
    printDetails();
    lastDetailsPrintTime = currentTime;
  }
}

void handleOpenCommand() {
  if (!direction) { // If currently closing, change to opening
    stopMotor();
    direction = true;
    runMotor(direction);
  } else if (!isRunning) { // If not running, start opening
    runMotor(direction);
  }
}

void handleCloseCommand() {
  if (direction) { // If currently opening, change to closing
    stopMotor();
    direction = false;
    runMotor(direction);
  } else if (!isRunning) { // If not running, start closing
    runMotor(direction);
  }
}

void handleEmergencyStop() {
  stopMotor();
}

void handlePhotoCellBarrier() {
  stopMotor();
  photoCellTriggered = true;
  direction = true; // Set direction to opening
  runMotor(direction);
}

void handlePauseMovement(bool pauseMovement) {
  static bool previousPauseState = HIGH;
  if (pauseMovement == LOW && previousPauseState == HIGH) {
    isPaused = !isPaused;
    if (isPaused) {
      stopMotor();
    } else {
      if (!isRunning) {
        runMotor(direction);
      }
    }
    saveStateIfChanged();
  }
  previousPauseState = pauseMovement;
}

void handleChangeDirectionAndRun(bool changeDirectionAndRun, unsigned long currentTime) {
  static bool previousChangeDirectionState = HIGH;
  if (changeDirectionAndRun == LOW && previousChangeDirectionState == HIGH) {
    if (!isPaused && currentTime - lastDirectionChangeTime >= directionChangeDelay) {
      stopMotor();
      direction = !direction;
      lastDirectionChangeTime = currentTime;
      lastMotorStartTime = currentTime + motorStartDelay; // Schedule motor start after delay
      saveStateIfChanged();
    }
  }
  previousChangeDirectionState = changeDirectionAndRun;
}

void handleLimitSwitches(bool closedLimitSwitch, bool openedLimitSwitch) {
  if (isRunning) {
    if (direction && openedLimitSwitch) {
      stopMotor();
    } else if (!direction && closedLimitSwitch) {
      stopMotor();
    }
  }
}

void resumeMotorIfDelayed(unsigned long currentTime) {
  if (!isRunning && !isPaused && (currentTime >= lastMotorStartTime)) {
    runMotor(direction);
  }
}

void runMotor(bool direction) {
  // Check if motor should run based on limit switches
  bool canRun = true;
  if (direction && digitalRead(openedLimitSwitchPin) == LOW) {
    canRun = false;
  } else if (!direction && digitalRead(closedLimitSwitchPin) == LOW) {
    canRun = false;
  }

  if (canRun) {
    digitalWrite(directionRelayPin, direction ? HIGH : LOW);
    digitalWrite(mainRelayPin, HIGH);
    isRunning = true;
    saveStateIfChanged();
  }
}

void stopMotor() {
  digitalWrite(mainRelayPin, LOW);
  isRunning = false;
  saveStateIfChanged();
}

void saveStateIfChanged() {
  bool eepromIsPaused = EEPROM.read(isPausedAddr);
  bool eepromIsRunning = EEPROM.read(isRunningAddr);
  bool eepromDirection = EEPROM.read(directionAddr);
  bool eepromPhotoCellTriggered = EEPROM.read(photoCellTriggeredAddr);

  if (isPaused != eepromIsPaused || isRunning != eepromIsRunning || direction != eepromDirection || photoCellTriggered != eepromPhotoCellTriggered) {
    EEPROM.write(isPausedAddr, isPaused);
    EEPROM.write(isRunningAddr, isRunning);
    EEPROM.write(directionAddr, direction);
    EEPROM.write(photoCellTriggeredAddr, photoCellTriggered);
    // Compute and store checksum
    byte checksum = computeChecksum();
    EEPROM.write(checksumAddr, checksum);
  }
}

bool readStateFromEEPROM() {
  isPaused = EEPROM.read(isPausedAddr);
  isRunning = EEPROM.read(isRunningAddr);
  direction = EEPROM.read(directionAddr);
  photoCellTriggered = EEPROM.read(photoCellTriggeredAddr);

  // Verify checksum
  byte storedChecksum = EEPROM.read(checksumAddr);
  byte computedChecksum = computeChecksum();

  return storedChecksum == computedChecksum;
}

byte computeChecksum() {
  return isPaused + isRunning + direction + photoCellTriggered;
}

void printDetails() {
  Serial.println("===== Door Control System Details =====");
  Serial.print("Paused: ");
  Serial.println(isPaused ? "Yes" : "No");
  Serial.print("Running: ");
  Serial.println(isRunning ? "Yes" : "No");
  Serial.print("Direction: ");
  Serial.println(direction ? "Opening" : "Closing");
  Serial.print("Photo Cell Triggered: ");
  Serial.println(photoCellTriggered ? "Yes" : "No");
  Serial.print("Closed Limit Switch: ");
  Serial.println(digitalRead(closedLimitSwitchPin) == LOW ? "Engaged" : "Not Engaged");
  Serial.print("Opened Limit Switch: ");
  Serial.println(digitalRead(openedLimitSwitchPin) == LOW ? "Engaged" : "Not Engaged");
  Serial.print("Emergency Stop: ");
  Serial.println(emergencyStopDebouncer.read() == LOW ? "Engaged" : "Not Engaged");
  Serial.print("Pause Movement Button: ");
  Serial.println(pauseMovementDebouncer.read() == LOW ? "Pressed" : "Not Pressed");
  Serial.print("Change Direction Button: ");
  Serial.println(changeDirectionAndRunDebouncer.read() == LOW ? "Pressed" : "Not Pressed");
  Serial.print("Photo Cell Barrier: ");
  Serial.println(photoCellBarrierDebouncer.read() == LOW ? "Triggered" : "Not Triggered");
  Serial.println("=======================================");
}
