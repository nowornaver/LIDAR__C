#ifndef RPLIDAR_H
#define RPLIDAR_H
#include <stdint.h>  // uint8_t와 같은 타입을 정의하기 위한 헤더 파일 추가
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include "rpLidarTypes.h"
#include <stdbool.h>
// Struct for rpLidar to hold necessary data
typedef struct {
    uint8_t scanMode;        ///< Current scan mode of lidar
    bool status;             ///< Current status of lidar
    uint16_t interestAngleLeft;  ///< Left border of needed angle 180-360°
    uint16_t interestAngleRight; ///< Right border of needed angle 0-180°

    point_t Data[3250]; ///< Stores the raw scan data
    stScanDataPoint_t DataBuffer[1500];  ///< Storage to save the Data of a Standard Scan
    stExpressDataPacket_t ExpressDataBuffer[79];  ///< Storage to save the Data of an Express Scan
} rpLidar_t;

/**
 * Constructor function for rpLidar struct
 *
 * @param _serial Pointer to the USART used
 * @param baud Baud rate for serial communication
 * @return A pointer to the rpLidar struct
 */


/**
 * Gets the device info from rpLidar
 *
 * @return Requested Device Info
 */
stDeviceInfo_t rpLidar_GetDeviceInfo(rpLidar_t *lidar);

/**
 * Gets the device health status from rpLidar
 *
 * @return Device health status
 */
stDeviceStatus_t rpLidar_GetDeviceHealth(rpLidar_t *lidar);

/**
 * Resets the lidar device
 */
void rpLidar_ResetDevice(rpLidar_t *lidar);

/**
 * Stops the lidar if it's turning
 */
void rpLidar_StopDevice(rpLidar_t *lidar);

/**
 * Starts the lidar and its measurement system
 *
 * @param mode The mode to run the lidar
 * @return true if mode started correctly, false otherwise
 */
bool rpLidar_Start(rpLidar_t *lidar, uint8_t mode);

/**
 * Reads the measurement points from the lidar
 *
 * @return The number of data points based on the scan mode (express *40, standard *1)
 */
uint16_t rpLidar_ReadMeasurePoints();

/**
 * Sets the angle of interest for scanning
 *
 * @param left Left angle border
 * @param right Right angle border
 */
void rpLidar_SetAngleOfInterest(rpLidar_t *lidar, uint16_t left, uint16_t right);

/**
 * Debug function to print measurement points in the serial monitor
 *
 * @param count Number of points to print
 */
void rpLidar_DebugPrintMeasurePoints(int16_t count);

/**
 * Debug function to print the device error status
 */
void rpLidar_DebugPrintDeviceErrorStatus(stDeviceStatus_t status);

/**
 * Debug function to print device info
 */
void rpLidar_DebugPrintDeviceInfo(stDeviceInfo_t info);

/**
 * Debug function to print descriptor information
 */
void rpLidar_DebugPrintDescriptor(rp_descriptor_t descriptor);

/**
 * Debug function to print the buffer in hexadecimal format
 */
void rpLidar_DebugPrintBufferAsHex(rpLidar_t *lidar);

/**
 * Compares two response descriptors
 *
 * @param descr1 First descriptor to compare
 * @param descr2 Second descriptor to compare
 * @return true if descriptors are equal
 */
bool rpLidar_CompareDescriptor(uint8_t *descr1, uint8_t *descr2);

/**
 * Clears the serial buffer
 */
void rpLidar_ClearSerialBuffer(rpLidar_t *lidar);

/**
 * Checks the CRC of an express scan packet
 *
 * @param package The express data packet
 * @param crc The expected CRC value
 * @return true if the CRC matches, false otherwise
 */
bool rpLidar_CheckCRC(stExpressDataPacket_t package, uint8_t crc);

/**
 * Checks if no serial data is available within the given time
 *
 * @param time Timeout period in milliseconds
 * @param size Expected number of bytes
 * @return true if a timeout occurs
 */
bool rpLidar_CheckForTimeout(rpLidar_t *lidar, uint32_t time, size_t size);

/**
 * Calculates the angle for each cabin in an express data packet
 *
 * @param packet Pointer to the packet
 * @param k Cabin index
 * @return Calculated angle
 */
float rpLidar_CalcAngle(uint8_t _lowByte, uint8_t _highByte);

/**
 * Converts express data to an array of points
 *
 * @param packets Pointer to lidar receive express data buffer
 * @param count Number of packets
 * @return true if successful, false if failed
 */
bool rpLidar_ExpressDataToPointArray(stExpressDataPacket_t *packets, uint16_t count);

/**
 * Awaits a standard scan and returns the number of points
 *
 * @return Number of points
 */
uint16_t rpLidar_awaitStandardScan();

/**
 * Awaits an express scan and returns the number of cabins
 *
 * @return Number of cabins
 */
uint16_t rpLidar_AwaitExpressScan(rpLidar_t *lidar);

/**
 * Starts an express scan directly
 *
 * @return Number of cabins with data
 */
uint16_t rpLidar_ScanExpress(rpLidar_t *lidar);

/**
 * Starts a standard scan directly
 *
 * @return Number of measurement points
 */
uint16_t rpLidar_ScanStandard();

/**
 * Checks if the angle of data is within the set range
 *
 * @param point The data point to check
 * @return true if the angle is between borders
 */
bool rpLidar_IsDataBetweenBorders(rpLidar_t *lidar, stScanDataPoint_t point);

/**
 * Checks if the angle of data is within the set range (overload)
 *
 * @param angle The angle to check
 * @return true if the angle is between borders
 */
bool rpLidar_IsDataBetweenBordersAngle(rpLidar_t *lidar, float angle);

/**
 * Calculates the angle for standard mode
 *
 * @param lowByte Low byte of the angle data
 * @param highByte High byte of the angle data
 * @return The calculated angle
 */
float rpLidar_CalcAngleStandard(uint8_t lowByte, uint8_t highByte);

/**
 * Calculates the angle for express mode
 *
 * @param angle1 First angle value
 * @param angle2 Second angle value
 * @param k Cabin index
 * @return The calculated angle
 */
float rpLidar_CalcExpressAngle(uint16_t angle1, uint16_t angle2, uint8_t k);

/**
 * Calculates the distance for standard mode
 *
 * @param lowByte Low byte of the distance data
 * @param highByte High byte of the distance data
 * @return The calculated distance
 */
float rpLidar_calcDistance(uint8_t lowByte, uint8_t highByte);

/**
 * Checks if the measured distance is valid
 *
 * @param point The scan data point to check
 * @return true if the data is valid
 */
bool rpLidar_IsDataValid(rpLidar_t *lidar, stScanDataPoint_t point);

/**
 * Checks if the measured distance is valid (overload)
 *
 * @param distance The distance to check
 * @return true if the data is valid
 */
bool rpLidar_IsDataValidDistance(uint16_t distance);

/**
 * Checks if the motor is running
 *
 * @return true if the motor is running
 */
bool rpLidar_IsRunning(rpLidar_t *lidar);

/**
 * Returns the current scan mode
 *
 * @return The current scan mode
 */
uint8_t rpLidar_GetScanMode(rpLidar_t *lidar);

#endif // RPLIDAR_H
