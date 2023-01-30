#include <sbgCommon.h>
#include <sbgEComLib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>

#pragma pack(1)
struct Generic_IMU_record_SI {
    int flag;
    int week;
    double ToW;     // Time of Week          [     s     ]
    float dw[3];   // Angular  increments   [    rad    ]
    float dv[3];   // Velocity increments   [    m/s    ]
};
#pragma pack()

int weekCal(int dayCurrent, int monthCurrent, int yearCurrent) {
    int dayStart = 5;
    int monthStart = 1;
    int yearStart = 1980;
    int days = 0;
    int daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (int i = yearStart; i < yearCurrent; i++) {
        if (i % 4 == 0) {
            days += 366;
        } else {
            days += 365;
        }
    }
    for (int i = 0; i < monthStart - 1; i++) {
        days -= daysInMonth[i];
    }
    days -= dayStart;
    for (int i = 0; i < monthCurrent - 1; i++) {
        days += daysInMonth[i];
    }
    days += dayCurrent;
    return days / 7;
}

typedef struct _UserArgs {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    uint32_t timeStamp;
    uint32_t gpsTimeOfWeek;
    bool use = false;
    std::ofstream m_table_out;
} UserArgs;

SbgErrorCode
onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData,
              void *pUserArg) {
    assert(pLogData);

    SBG_UNUSED_PARAMETER(pHandle);
    auto *userArgs = (UserArgs *) pUserArg;


    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0) {
        switch (msg) {
            case SBG_ECOM_LOG_IMU_DATA:
                if (userArgs->use) {
                    Generic_IMU_record_SI tp_imu;
                    tp_imu.flag = 0;
                    tp_imu.week = weekCal(userArgs->day, userArgs->month, userArgs->year);
                    tp_imu.ToW = ((double) userArgs->gpsTimeOfWeek / 1000.0) +
                                 (double) (pLogData->imuData.timeStamp - userArgs->timeStamp) / 1000000.0;
                    tp_imu.dw[0] = pLogData->imuData.deltaAngle[0] / 200;
                    tp_imu.dw[1] = pLogData->imuData.deltaAngle[1] / 200;
                    tp_imu.dw[2] = pLogData->imuData.deltaAngle[2] / 200;

                    tp_imu.dv[0] = pLogData->imuData.deltaVelocity[0] / 200;
                    tp_imu.dv[1] = pLogData->imuData.deltaVelocity[1] / 200;
                    tp_imu.dv[2] = pLogData->imuData.deltaVelocity[2] / 200;
                    if (userArgs->year == 2023 && userArgs->month == 1 && userArgs->day == 18 && userArgs->hour == 13 && userArgs->minute == 22 && userArgs->second == 22){
                        std::cout<<std::setprecision(10)<<tp_imu.dv[1]<<std::endl;
                    }
                    if (userArgs->m_table_out.is_open()) {
                        userArgs->m_table_out.write(reinterpret_cast<char *>(&tp_imu), sizeof(Generic_IMU_record_SI));
                    }
                }
                break;
            case SBG_ECOM_LOG_UTC_TIME:
                userArgs->use = true;
                userArgs->year = pLogData->utcData.year;
                userArgs->month = pLogData->utcData.month;
                userArgs->day = pLogData->utcData.day;
                userArgs->timeStamp = pLogData->utcData.timeStamp;
                userArgs->gpsTimeOfWeek = pLogData->utcData.gpsTimeOfWeek;
                userArgs->hour = pLogData->utcData.hour;
                userArgs->minute = pLogData->utcData.minute;
                userArgs->second = pLogData->utcData.second;

                break;
            default:
                break;
        }
    }
    return SBG_NO_ERROR;
}

static SbgErrorCode FileInputProcess(SbgInterface *pInterface) {
    SbgErrorCode errorCode = SBG_NO_ERROR;
    SbgEComHandle comHandle;
    UserArgs userArgs;
    userArgs.m_table_out.open("test.sbgimu", std::ios::binary);


    assert(pInterface);

    errorCode = sbgEComInit(&comHandle, pInterface);

    if (errorCode == SBG_NO_ERROR) {

        sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, &userArgs);


        while (1) {
            errorCode = sbgEComHandle(&comHandle);
            if (errorCode == SBG_NOT_READY) {

                sbgSleep(1);
            } else {
                SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");
            }
        }
        sbgEComClose(&comHandle);
    } else {
        SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
    }
    return errorCode;
}

int main(int argc, char **argv) {
    SbgErrorCode errorCode = SBG_NO_ERROR;
    SbgInterface sbgInterface;
    int exitCode;


    SBG_UNUSED_PARAMETER(argc);
    SBG_UNUSED_PARAMETER(argv);



    errorCode = sbgInterfaceSerialCreate(&sbgInterface, argv[1], atoi(argv[2]));

    if (errorCode == SBG_NO_ERROR) {
        errorCode = FileInputProcess(&sbgInterface);

        if (errorCode == SBG_NO_ERROR) {
            exitCode = EXIT_SUCCESS;
        } else {
            exitCode = EXIT_FAILURE;
        }

        sbgInterfaceDestroy(&sbgInterface);
    } else {
        SBG_LOG_ERROR(errorCode, "unable to open serial interface");
        exitCode = EXIT_FAILURE;
    }

    return exitCode;
}
