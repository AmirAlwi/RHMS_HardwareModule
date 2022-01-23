#include "Arduino.h"

#define dateLoc "fields/date/"

#define distanceLoc "fields/distance/"

#define notesLoc "fields/notes/"

#define titleLoc "fields/title/"

#define uidLoc "fields/uid/"

#define starttimeLoc "fields/time/mapValue/fields/starttime/"
#define endtimeLoc "fields/time/mapValue/fields/endtime/"

#define sensorArrLoc "fields/sensor/arrayValue/value/"

#define heartrateLoc "fields/sensordata/mapValue/fields/heartrate/arrayValue/value/"
#define oximeterLoc "fields/sensordata/mapValue/fields/oximeter/arrayValue/value/"
#define temperatureLoc "fields/sensordata/mapValue/fields/temperature/arrayValue/value/"
#define postureLoc "fields/sensordata/mapValue/fields/posture/arrayValue/value/"
#define bpUpLoc "fields/sensordata/mapValue/fields/bloodpressure/mapValue/fields/upper/arrayValue/value/"
#define bpLowLoc "fields/sensordata/mapValue/fields/bloodpressure/mapValue/fields/lower/arrayValue/value/"

//for testing mpu9250
#define yawLoc "fields/sensor/mapValue/fields/orientation/mapValue/fields/yaw/arrayValue/values/"
#define rollLoc "fields/sensor/mapValue/fields/orientation/mapValue/fields/roll/arrayValue/values/"
#define pitchLoc "fields/sensor/mapValue/fields/orientation/mapValue/fields/pitch/arrayValue/values/"