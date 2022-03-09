#include "Arduino.h"

#define dateLoc "fields/date/"

#define distanceLoc "fields/distance/"

#define notesLoc "fields/notes/"

#define titleLoc "fields/title/"

#define uidLoc "fields/uid/"

#define starttimeLoc "fields/time/mapValue/fields/starttime/"
#define endtimeLoc "fields/time/mapValue/fields/endtime/"

#define sensorArrLoc "fields/sensor/arrayValue/values/"

#define heartrateLoc "fields/sensordata/mapValue/fields/heartrate/arrayValue/values/"
#define oximeterLoc "fields/sensordata/mapValue/fields/oximeter/arrayValue/values/"
#define temperatureLoc "fields/sensordata/mapValue/fields/temperature/arrayValue/values/"
#define postureLoc "fields/sensordata/mapValue/fields/posture/arrayValue/values/"
#define bpUpLoc "fields/sensordata/mapValue/fields/bloodpressure/mapValue/fields/upper/doubleValue"
#define bpLowLoc "fields/sensordata/mapValue/fields/bloodpressure/mapValue/fields/lower/doubleValue"
#define position_latitudeLoc "fields/sensordata/mapValue/fields/position/geoPointValue/latitude"
#define position_logitudeLoc "fields/sensordata/mapValue/fields/position/geoPointValue/longitude"
//for testing mpu9250
#define yawLoc "fields/sensordata/mapValue/fields/orientation/mapValue/fields/x/arrayValue/values/"
#define rollLoc "fields/sensordata/mapValue/fields/orientation/mapValue/fields/y/arrayValue/values/"
#define pitchLoc "fields/sensordata/mapValue/fields/orientation/mapValue/fields/z/arrayValue/values/"

#define accXLoc "fields/sensordata/mapValue/fields/acceleration/mapValue/fields/x/arrayValue/values/"
#define accYLoc "fields/sensordata/mapValue/fields/acceleration/mapValue/fields/y/arrayValue/values/"
#define accZLoc "fields/sensordata/mapValue/fields/acceleration/mapValue/fields/z/arrayValue/values/"

#define magXLoc "fields/sensordata/mapValue/fields/magnet/mapValue/fields/x/arrayValue/values/"
#define magYLoc "fields/sensordata/mapValue/fields/magnet/mapValue/fields/y/arrayValue/values/"
#define magZLoc "fields/sensordata/mapValue/fields/magnet/mapValue/fields/z/arrayValue/values/"

#define gyroXLoc "fields/sensordata/mapValue/fields/gyro/mapValue/fields/x/arrayValue/values/"
#define gyroYLoc "fields/sensordata/mapValue/fields/gyro/mapValue/fields/y/arrayValue/values/"
#define gyroZLoc "fields/sensordata/mapValue/fields/gyro/mapValue/fields/z/arrayValue/values/"