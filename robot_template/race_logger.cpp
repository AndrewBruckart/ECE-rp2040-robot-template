#include "race_logger.h"
#include "sensors.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

static const unsigned long RACE_LOG_SAMPLE_MS = 200;
static const size_t RACE_LOG_MAX_RECORDS = 1600;
static const size_t SERIAL_COMMAND_BUFFER_SIZE = 96;
static const int INVALID_STATE = -1;

enum RaceLogEvent {
  RACE_LOG_EVENT_START = 0,
  RACE_LOG_EVENT_SAMPLE,
  RACE_LOG_EVENT_STATE,
  RACE_LOG_EVENT_STOP
};

struct RaceLogRecord {
  unsigned long elapsedMs;
  uint8_t event;
  uint8_t mode;
  uint8_t reason;
  int8_t stepIndex;
  int8_t currentState;
  int8_t previousState;
  int8_t nextState;
  uint8_t wallSide;
  uint8_t motorRunning;
  int8_t driveCommand;
  int16_t steeringAngle;
  int8_t motorSpeedPercent;
  float targetWallDistanceInches;
  float wallKp;
  float leftDistanceInches;
  float centerDistanceInches;
  float rightDistanceInches;
  float activeWallDistanceInches;
  float wallErrorInches;
  float controlOutputDegrees;
  uint16_t centerRawAdc;
  uint16_t ldrRaw;
  int16_t accelXMg;
  int16_t accelYMg;
  int16_t accelZMg;
};

struct RaceLogSession {
  bool fileSystemReady;
  bool active;
  bool truncated;
  unsigned long startedAtMs;
  unsigned long lastSampleAtMs;
  size_t recordCount;
  RaceLogMode mode;
  int stepIndex;
  WallFollowState lastLoggedState;
};

struct FileListEntry {
  char path[32];
  size_t size;
  int logIndex;
};

static RaceLogSession loggerSession = {};
static RaceLogRecord logRecords[RACE_LOG_MAX_RECORDS];
static char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
static size_t serialCommandLength = 0;
static bool littleFsFormatAttempted = false;

static const char *eventName(RaceLogEvent event) {
  switch (event) {
    case RACE_LOG_EVENT_START: return "START";
    case RACE_LOG_EVENT_STATE: return "STATE";
    case RACE_LOG_EVENT_STOP: return "STOP";
    case RACE_LOG_EVENT_SAMPLE:
    default:
      return "SAMPLE";
  }
}

static const char *modeName(RaceLogMode mode) {
  switch (mode) {
    case RACE_LOG_MODE_RACE: return "RACE";
    case RACE_LOG_MODE_STEP: return "STEP";
    case RACE_LOG_MODE_NONE:
    default:
      return "";
  }
}

static const char *stopReasonName(RaceLogStopReason reason) {
  switch (reason) {
    case RACE_LOG_STOP_REASON_STOP_BUTTON: return "stop_button";
    case RACE_LOG_STOP_REASON_EXIT_SCREEN: return "exit_screen";
    case RACE_LOG_STOP_REASON_MODE_CHANGED: return "mode_changed";
    case RACE_LOG_STOP_REASON_COMPLETE: return "complete";
    case RACE_LOG_STOP_REASON_NONE:
    default:
      return "";
  }
}

static void readAcceleration(int &x, int &y, int &z) {
  x = 0;
  y = 0;
  z = 0;
  if (!accelAvailable()) {
    return;
  }
  readAccelXYZ(x, y, z);
}

static bool ensureLittleFSReady(bool allowFormat) {
  if (loggerSession.fileSystemReady) {
    return true;
  }

  if (LittleFS.begin()) {
    loggerSession.fileSystemReady = true;
    return true;
  }

  if (allowFormat && !littleFsFormatAttempted) {
    littleFsFormatAttempted = true;
    Serial.println("WARN LOGGER formatting_littlefs");
    if (LittleFS.format() && LittleFS.begin()) {
      loggerSession.fileSystemReady = true;
      Serial.println("OK LOGGER fs_ready_after_format");
      return true;
    }
  }

  Serial.println("ERR LOGGER fs_init_failed check_flash_layout=flash=16777216_8388608");
  loggerSession.fileSystemReady = false;
  return false;
}

static unsigned long currentElapsedMs() {
  return millis() - loggerSession.startedAtMs;
}

static void resetSessionState() {
  loggerSession.active = false;
  loggerSession.truncated = false;
  loggerSession.startedAtMs = 0;
  loggerSession.lastSampleAtMs = 0;
  loggerSession.recordCount = 0;
  loggerSession.mode = RACE_LOG_MODE_NONE;
  loggerSession.stepIndex = -1;
  loggerSession.lastLoggedState = WALL_FOLLOW_STATE_TRACKING;
}

static void fillRecord(
  RaceLogRecord &record,
  RaceLogEvent event,
  RaceLogStopReason reason,
  const WallFollowStatus &status,
  const WallFollowTuning &tuning,
  bool motorRunning,
  int previousState,
  int nextState
) {
  int accelX = 0;
  int accelY = 0;
  int accelZ = 0;
  readAcceleration(accelX, accelY, accelZ);

  record.elapsedMs = currentElapsedMs();
  record.event = (uint8_t)event;
  record.mode = (uint8_t)loggerSession.mode;
  record.reason = (uint8_t)reason;
  record.stepIndex = (int8_t)loggerSession.stepIndex;
  record.currentState = (int8_t)status.state;
  record.previousState = (int8_t)previousState;
  record.nextState = (int8_t)nextState;
  record.wallSide = (uint8_t)status.selectedWall;
  record.motorRunning = motorRunning ? 1 : 0;
  record.driveCommand = (int8_t)status.driveCommand;
  record.steeringAngle = (int16_t)status.steeringAngle;
  record.motorSpeedPercent = (int8_t)tuning.motorSpeedPercent;
  record.targetWallDistanceInches = tuning.targetWallDistanceInches;
  record.wallKp = tuning.kp;
  record.leftDistanceInches = status.leftDistanceInches;
  record.centerDistanceInches = status.centerDistanceInches;
  record.rightDistanceInches = status.rightDistanceInches;
  record.activeWallDistanceInches = status.activeWallDistanceInches;
  record.wallErrorInches = status.wallErrorInches;
  record.controlOutputDegrees = status.controlOutputDegrees;
  record.centerRawAdc = (uint16_t)status.centerRawAdc;
  record.ldrRaw = (uint16_t)readLDR();
  record.accelXMg = (int16_t)accelX;
  record.accelYMg = (int16_t)accelY;
  record.accelZMg = (int16_t)accelZ;
}

static bool appendRecord(
  RaceLogEvent event,
  RaceLogStopReason reason,
  const WallFollowStatus &status,
  const WallFollowTuning &tuning,
  bool motorRunning,
  int previousState,
  int nextState,
  bool force
) {
  if (!loggerSession.active) {
    return false;
  }

  if (loggerSession.recordCount >= RACE_LOG_MAX_RECORDS) {
    loggerSession.truncated = true;
    if (!force) {
      return false;
    }
    loggerSession.recordCount = RACE_LOG_MAX_RECORDS - 1;
  }

  RaceLogRecord &record = logRecords[loggerSession.recordCount];
  fillRecord(record, event, reason, status, tuning, motorRunning, previousState, nextState);
  if (loggerSession.recordCount < RACE_LOG_MAX_RECORDS) {
    loggerSession.recordCount++;
  }
  return true;
}

static bool isRaceLogName(const char *path) {
  if (path == NULL) {
    return false;
  }

  const char *name = path;
  if (name[0] == '/') {
    name++;
  }

  if (strncmp(name, "race_", 5) != 0) {
    return false;
  }
  if (strlen(name) != 13) {
    return false;
  }
  if (strcmp(name + 9, ".csv") != 0) {
    return false;
  }
  for (int i = 5; i < 9; ++i) {
    if (!isdigit((unsigned char)name[i])) {
      return false;
    }
  }
  return true;
}

static int raceLogIndexFromPath(const char *path) {
  if (!isRaceLogName(path)) {
    return -1;
  }

  const char *name = path;
  if (name[0] == '/') {
    name++;
  }

  return (name[5] - '0') * 1000 +
    (name[6] - '0') * 100 +
    (name[7] - '0') * 10 +
    (name[8] - '0');
}

static void formatRaceLogPath(int index, char *buffer, size_t bufferSize) {
  if (bufferSize == 0) {
    return;
  }
  snprintf(buffer, bufferSize, "/race_%04d.csv", index);
}

static bool normalizePath(const char *input, char *buffer, size_t bufferSize) {
  if (input == NULL || buffer == NULL || bufferSize < 2) {
    return false;
  }

  while (*input != '\0' && isspace((unsigned char)*input)) {
    input++;
  }

  size_t length = strlen(input);
  while (length > 0 && isspace((unsigned char)input[length - 1])) {
    length--;
  }

  if (length == 0 || length >= bufferSize) {
    return false;
  }

  size_t out = 0;
  if (input[0] != '/') {
    buffer[out++] = '/';
  }

  for (size_t i = 0; i < length && out < bufferSize - 1; ++i) {
    char ch = input[i];
    if (ch == '\\') {
      ch = '/';
    }
    buffer[out++] = ch;
  }
  buffer[out] = '\0';

  if (strstr(buffer, "..") != NULL) {
    return false;
  }

  return true;
}

static size_t collectRootFiles(FileListEntry *entries, size_t maxEntries, bool logsOnly, int *latestLogIndex, char *latestLogPath, size_t latestLogPathSize) {
  if (latestLogIndex != NULL) {
    *latestLogIndex = -1;
  }
  if (latestLogPath != NULL && latestLogPathSize > 0) {
    latestLogPath[0] = '\0';
  }

  File root = LittleFS.open("/", "r");
  if (!root) {
    return 0;
  }

  size_t count = 0;
  File entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      const char *name = entry.name();
      int logIndex = raceLogIndexFromPath(name);
      bool include = !logsOnly || logIndex >= 0;
      if (include) {
        if (entries != NULL && count < maxEntries) {
          strncpy(entries[count].path, name, sizeof(entries[count].path) - 1);
          entries[count].path[sizeof(entries[count].path) - 1] = '\0';
          entries[count].size = entry.size();
          entries[count].logIndex = logIndex;
        }
        count++;
      }
      if (logIndex >= 0 && latestLogIndex != NULL && logIndex > *latestLogIndex) {
        *latestLogIndex = logIndex;
        if (latestLogPath != NULL && latestLogPathSize > 0) {
          strncpy(latestLogPath, name, latestLogPathSize - 1);
          latestLogPath[latestLogPathSize - 1] = '\0';
        }
      }
    }
    entry.close();
    entry = root.openNextFile();
  }

  root.close();
  return count;
}

static bool writeCsvHeader(File &file) {
  file.println(
    "elapsed_ms,event,mode,step_index,wall_side,current_state,previous_state,next_state,"
    "motor_running,drive_command,motor_speed_percent,steering_angle,target_wall_distance_in,"
    "wall_kp,left_distance_in,center_distance_in,right_distance_in,active_wall_distance_in,"
    "wall_error_in,control_output_deg,center_raw_adc,ldr_raw,accel_x_mg,accel_y_mg,accel_z_mg,"
    "stop_reason,buffer_truncated"
  );
  return file;
}

static void writeStateField(File &file, int stateValue) {
  if (stateValue == INVALID_STATE) {
    return;
  }
  file.print(wallFollowStateName((WallFollowState)stateValue));
}

static void writeStepIndexField(File &file, int stepIndex) {
  if (stepIndex < 0) {
    return;
  }
  file.print(stepIndex);
}

static bool writeCsvRecord(File &file, const RaceLogRecord &record) {
  file.print(record.elapsedMs);
  file.print(',');
  file.print(eventName((RaceLogEvent)record.event));
  file.print(',');
  file.print(modeName((RaceLogMode)record.mode));
  file.print(',');
  writeStepIndexField(file, record.stepIndex);
  file.print(',');
  file.print(wallFollowSideName((WallFollowSide)record.wallSide));
  file.print(',');
  file.print(wallFollowStateName((WallFollowState)record.currentState));
  file.print(',');
  writeStateField(file, record.previousState);
  file.print(',');
  writeStateField(file, record.nextState);
  file.print(',');
  file.print(record.motorRunning ? 1 : 0);
  file.print(',');
  file.print(record.driveCommand);
  file.print(',');
  file.print(record.motorSpeedPercent);
  file.print(',');
  file.print(record.steeringAngle);
  file.print(',');
  file.print(record.targetWallDistanceInches, 2);
  file.print(',');
  file.print(record.wallKp, 2);
  file.print(',');
  file.print(record.leftDistanceInches, 2);
  file.print(',');
  file.print(record.centerDistanceInches, 2);
  file.print(',');
  file.print(record.rightDistanceInches, 2);
  file.print(',');
  file.print(record.activeWallDistanceInches, 2);
  file.print(',');
  file.print(record.wallErrorInches, 2);
  file.print(',');
  file.print(record.controlOutputDegrees, 2);
  file.print(',');
  file.print(record.centerRawAdc);
  file.print(',');
  file.print(record.ldrRaw);
  file.print(',');
  file.print(record.accelXMg);
  file.print(',');
  file.print(record.accelYMg);
  file.print(',');
  file.print(record.accelZMg);
  file.print(',');
  file.print(stopReasonName((RaceLogStopReason)record.reason));
  file.print(',');
  file.println(loggerSession.truncated ? 1 : 0);
  return file;
}

static bool saveLogToLittleFS() {
  if (!loggerSession.fileSystemReady || loggerSession.recordCount == 0) {
    return false;
  }

  int latestIndex = -1;
  collectRootFiles(NULL, 0, true, &latestIndex, NULL, 0);

  char logPath[20];
  formatRaceLogPath(latestIndex + 1, logPath, sizeof(logPath));

  File file = LittleFS.open(logPath, "w");
  if (!file) {
    Serial.println("ERR LOGGER open_failed");
    return false;
  }

  bool ok = writeCsvHeader(file);
  for (size_t i = 0; ok && i < loggerSession.recordCount; ++i) {
    ok = writeCsvRecord(file, logRecords[i]);
  }

  file.flush();
  ok = ok && file;
  file.close();

  if (!ok) {
    LittleFS.remove(logPath);
    Serial.println("ERR LOGGER write_failed");
    return false;
  }

  Serial.print("OK LOGGER saved path=");
  Serial.print(logPath);
  Serial.print(" rows=");
  Serial.println((unsigned long)loggerSession.recordCount);
  return true;
}

static void printHelp() {
  Serial.println("OK HELP count=6");
  Serial.println("CMD HELP");
  Serial.println("CMD LISTLOGS");
  Serial.println("CMD DUMPLATEST");
  Serial.println("CMD DUMPLOG /race_0000.csv");
  Serial.println("CMD LISTFILES");
  Serial.println("CMD DUMPFILE /name.txt");
  Serial.println("END HELP");
}

static void printFiles(bool logsOnly) {
  if (!ensureLittleFSReady(false)) {
    Serial.println("ERR FS unavailable");
    return;
  }

  FileListEntry entries[64];
  size_t count = collectRootFiles(entries, 64, logsOnly, NULL, NULL, 0);
  if (logsOnly) {
    Serial.print("OK LISTLOGS count=");
  } else {
    Serial.print("OK LISTFILES count=");
  }
  Serial.println((unsigned long)count);

  size_t listedCount = min(count, sizeof(entries) / sizeof(entries[0]));
  for (size_t i = 0; i < listedCount; ++i) {
    Serial.print(logsOnly ? "LOG path=" : "FILE path=");
    Serial.print(entries[i].path);
    Serial.print(" size=");
    Serial.println((unsigned long)entries[i].size);
  }

  Serial.println(logsOnly ? "END LISTLOGS" : "END LISTFILES");
}

static void dumpFileContents(const char *path, const char *commandName) {
  if (!ensureLittleFSReady(false)) {
    Serial.println("ERR FS unavailable");
    return;
  }

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.print("ERR ");
    Serial.print(commandName);
    Serial.print(" not_found path=");
    Serial.println(path);
    return;
  }

  Serial.print("OK ");
  Serial.print(commandName);
  Serial.print(" path=");
  Serial.print(path);
  Serial.print(" size=");
  Serial.println((unsigned long)file.size());
  Serial.print("BEGIN FILE ");
  Serial.println(path);

  while (file.available()) {
    Serial.write(file.read());
  }
  if (file.size() > 0) {
    Serial.println();
  }

  Serial.print("END FILE ");
  Serial.println(path);
  file.close();
}

static void handleDumpLatest() {
  if (!ensureLittleFSReady(false)) {
    Serial.println("ERR FS unavailable");
    return;
  }

  int latestIndex = -1;
  char latestPath[32];
  collectRootFiles(NULL, 0, true, &latestIndex, latestPath, sizeof(latestPath));
  if (latestIndex < 0 || latestPath[0] == '\0') {
    Serial.println("ERR DUMPLATEST no_logs");
    return;
  }

  dumpFileContents(latestPath, "DUMPLATEST");
}

static void handleDumpPath(const char *pathText, bool logsOnly, const char *commandName) {
  if (pathText == NULL || pathText[0] == '\0') {
    Serial.print("ERR ");
    Serial.print(commandName);
    Serial.println(" missing_path");
    return;
  }

  char path[32];
  if (!normalizePath(pathText, path, sizeof(path))) {
    Serial.print("ERR ");
    Serial.print(commandName);
    Serial.println(" invalid_path");
    return;
  }

  if (logsOnly && !isRaceLogName(path)) {
    Serial.print("ERR ");
    Serial.print(commandName);
    Serial.println(" invalid_log_name");
    return;
  }

  dumpFileContents(path, commandName);
}

static void uppercaseAscii(char *text) {
  if (text == NULL) {
    return;
  }
  while (*text != '\0') {
    *text = (char)toupper((unsigned char)*text);
    text++;
  }
}

static void handleSerialCommand(char *line) {
  while (*line != '\0' && isspace((unsigned char)*line)) {
    line++;
  }
  if (*line == '\0') {
    return;
  }

  char *args = strchr(line, ' ');
  if (args != NULL) {
    *args = '\0';
    args++;
    while (*args != '\0' && isspace((unsigned char)*args)) {
      args++;
    }
  }

  uppercaseAscii(line);

  if (strcmp(line, "HELP") == 0) {
    printHelp();
    return;
  }

  if (loggerSession.active) {
    Serial.println("ERR BUSY run_active");
    return;
  }

  if (strcmp(line, "LISTLOGS") == 0) {
    printFiles(true);
  } else if (strcmp(line, "LISTFILES") == 0) {
    printFiles(false);
  } else if (strcmp(line, "DUMPLATEST") == 0) {
    handleDumpLatest();
  } else if (strcmp(line, "DUMPLOG") == 0) {
    handleDumpPath(args, true, "DUMPLOG");
  } else if (strcmp(line, "DUMPFILE") == 0) {
    handleDumpPath(args, false, "DUMPFILE");
  } else {
    Serial.print("ERR UNKNOWN command=");
    Serial.println(line);
  }
}

void initRaceLogger() {
  resetSessionState();
  loggerSession.fileSystemReady = false;
  littleFsFormatAttempted = false;
  ensureLittleFSReady(true);
}

bool raceLoggerReady() {
  return loggerSession.fileSystemReady;
}

void beginRaceLog(RaceLogMode mode, int stepIndex, const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning) {
  if (mode == RACE_LOG_MODE_NONE || !ensureLittleFSReady(false)) {
    return;
  }

  resetSessionState();
  loggerSession.active = true;
  loggerSession.startedAtMs = millis();
  loggerSession.lastSampleAtMs = 0;
  loggerSession.mode = mode;
  loggerSession.stepIndex = stepIndex;
  loggerSession.lastLoggedState = status.state;

  appendRecord(
    RACE_LOG_EVENT_START,
    RACE_LOG_STOP_REASON_NONE,
    status,
    tuning,
    motorRunning,
    INVALID_STATE,
    status.state,
    false
  );
}

void serviceRaceLog(const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning) {
  if (!loggerSession.active) {
    return;
  }

  unsigned long now = millis();
  if (status.state != loggerSession.lastLoggedState) {
    appendRecord(
      RACE_LOG_EVENT_STATE,
      RACE_LOG_STOP_REASON_NONE,
      status,
      tuning,
      motorRunning,
      loggerSession.lastLoggedState,
      status.state,
      false
    );
    loggerSession.lastLoggedState = status.state;
  }

  if (now - loggerSession.lastSampleAtMs >= RACE_LOG_SAMPLE_MS) {
    if (appendRecord(
      RACE_LOG_EVENT_SAMPLE,
      RACE_LOG_STOP_REASON_NONE,
      status,
      tuning,
      motorRunning,
      INVALID_STATE,
      INVALID_STATE,
      false
    )) {
      loggerSession.lastSampleAtMs = now;
    }
  }
}

void endRaceLog(const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning, RaceLogStopReason reason) {
  if (!loggerSession.active) {
    return;
  }

  appendRecord(
    RACE_LOG_EVENT_STOP,
    reason,
    status,
    tuning,
    motorRunning,
    status.state,
    INVALID_STATE,
    true
  );

  saveLogToLittleFS();
  bool fsReady = loggerSession.fileSystemReady;
  resetSessionState();
  loggerSession.fileSystemReady = fsReady;
}

void setRaceLogStepIndex(int stepIndex) {
  if (!loggerSession.active) {
    return;
  }
  loggerSession.stepIndex = stepIndex;
}

void serviceRaceLoggerSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';
      handleSerialCommand(serialCommandBuffer);
      serialCommandLength = 0;
      continue;
    }
    if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = ch;
    } else {
      serialCommandLength = 0;
      Serial.println("ERR BUFFER command_too_long");
    }
  }
}
