/* ***********************************************************
 * This file was automatically generated on 2019-05-21.      *
 *                                                           *
 * C/C++ Bindings Version 2.1.25                             *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/

#ifndef BRICKLET_REAL_TIME_CLOCK_V2_H
#define BRICKLET_REAL_TIME_CLOCK_V2_H

#include "ip_connection.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup BrickletRealTimeClockV2 Real-Time Clock Bricklet 2.0
 */

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Battery-backed real-time clock
 */
typedef Device RealTimeClockV2;

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_DATE_TIME 1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_DATE_TIME 2

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_TIMESTAMP 3

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_OFFSET 4

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_OFFSET 5

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_DATE_TIME_CALLBACK_CONFIGURATION 6

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_DATE_TIME_CALLBACK_CONFIGURATION 7

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_ALARM 8

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_ALARM 9

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_SPITFP_ERROR_COUNT 234

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_BOOTLOADER_MODE 235

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_BOOTLOADER_MODE 236

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER 237

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_WRITE_FIRMWARE 238

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_SET_STATUS_LED_CONFIG 239

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_STATUS_LED_CONFIG 240

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_CHIP_TEMPERATURE 242

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_RESET 243

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_WRITE_UID 248

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_READ_UID 249

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_FUNCTION_GET_IDENTITY 255

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Signature: \code void callback(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond, uint8_t weekday, int64_t timestamp, void *user_data) \endcode
 * 
 * This callback is triggered periodically with the period that is set by
 * {@link real_time_clock_v2_set_date_time_callback_configuration}. The parameters are the
 * same as for {@link real_time_clock_v2_get_date_time}.
 */
#define REAL_TIME_CLOCK_V2_CALLBACK_DATE_TIME 10

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Signature: \code void callback(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond, uint8_t weekday, int64_t timestamp, void *user_data) \endcode
 * 
 * This callback is triggered every time the current date and time matches the
 * configured alarm (see {@link real_time_clock_v2_set_alarm}). The parameters are the same
 * as for {@link real_time_clock_v2_get_date_time}.
 */
#define REAL_TIME_CLOCK_V2_CALLBACK_ALARM 11


/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_MONDAY 1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_TUESDAY 2

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_WEDNESDAY 3

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_THURSDAY 4

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_FRIDAY 5

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_SATURDAY 6

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_WEEKDAY_SUNDAY 7

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_ALARM_MATCH_DISABLED -1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_ALARM_INTERVAL_DISABLED -1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_MODE_BOOTLOADER 0

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_MODE_FIRMWARE 1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_MODE_BOOTLOADER_WAIT_FOR_REBOOT 2

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_REBOOT 3

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT 4

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_OK 0

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_INVALID_MODE 1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_NO_CHANGE 2

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_ENTRY_FUNCTION_NOT_PRESENT 3

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_DEVICE_IDENTIFIER_INCORRECT 4

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_BOOTLOADER_STATUS_CRC_MISMATCH 5

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_STATUS_LED_CONFIG_OFF 0

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_STATUS_LED_CONFIG_ON 1

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_STATUS_LED_CONFIG_SHOW_HEARTBEAT 2

/**
 * \ingroup BrickletRealTimeClockV2
 */
#define REAL_TIME_CLOCK_V2_STATUS_LED_CONFIG_SHOW_STATUS 3

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * This constant is used to identify a Real-Time Clock Bricklet 2.0.
 *
 * The {@link real_time_clock_v2_get_identity} function and the
 * {@link IPCON_CALLBACK_ENUMERATE} callback of the IP Connection have a
 * \c device_identifier parameter to specify the Brick's or Bricklet's type.
 */
#define REAL_TIME_CLOCK_V2_DEVICE_IDENTIFIER 2106

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * This constant represents the display name of a Real-Time Clock Bricklet 2.0.
 */
#define REAL_TIME_CLOCK_V2_DEVICE_DISPLAY_NAME "Real-Time Clock Bricklet 2.0"

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Creates the device object \c real_time_clock_v2 with the unique device ID \c uid and adds
 * it to the IPConnection \c ipcon.
 */
void real_time_clock_v2_create(RealTimeClockV2 *real_time_clock_v2, const char *uid, IPConnection *ipcon);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Removes the device object \c real_time_clock_v2 from its IPConnection and destroys it.
 * The device object cannot be used anymore afterwards.
 */
void real_time_clock_v2_destroy(RealTimeClockV2 *real_time_clock_v2);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the response expected flag for the function specified by the
 * \c function_id parameter. It is *true* if the function is expected to
 * send a response, *false* otherwise.
 *
 * For getter functions this is enabled by default and cannot be disabled,
 * because those functions will always send a response. For callback
 * configuration functions it is enabled by default too, but can be disabled
 * via the real_time_clock_v2_set_response_expected function. For setter functions it is
 * disabled by default and can be enabled.
 *
 * Enabling the response expected flag for a setter function allows to
 * detect timeouts and other error conditions calls of this setter as well.
 * The device will then send a response for this purpose. If this flag is
 * disabled for a setter function then no response is send and errors are
 * silently ignored, because they cannot be detected.
 */
int real_time_clock_v2_get_response_expected(RealTimeClockV2 *real_time_clock_v2, uint8_t function_id, bool *ret_response_expected);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Changes the response expected flag of the function specified by the
 * \c function_id parameter. This flag can only be changed for setter
 * (default value: *false*) and callback configuration functions
 * (default value: *true*). For getter functions it is always enabled.
 *
 * Enabling the response expected flag for a setter function allows to detect
 * timeouts and other error conditions calls of this setter as well. The device
 * will then send a response for this purpose. If this flag is disabled for a
 * setter function then no response is send and errors are silently ignored,
 * because they cannot be detected.
 */
int real_time_clock_v2_set_response_expected(RealTimeClockV2 *real_time_clock_v2, uint8_t function_id, bool response_expected);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Changes the response expected flag for all setter and callback configuration
 * functions of this device at once.
 */
int real_time_clock_v2_set_response_expected_all(RealTimeClockV2 *real_time_clock_v2, bool response_expected);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Registers the given \c function with the given \c callback_id. The
 * \c user_data will be passed as the last parameter to the \c function.
 */
void real_time_clock_v2_register_callback(RealTimeClockV2 *real_time_clock_v2, int16_t callback_id, void *function, void *user_data);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the API version (major, minor, release) of the bindings for this
 * device.
 */
int real_time_clock_v2_get_api_version(RealTimeClockV2 *real_time_clock_v2, uint8_t ret_api_version[3]);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the current date (including weekday) and the current time with hundredths
 * of a second resolution.
 * 
 * Possible value ranges:
 * 
 * * Year: 2000 to 2099
 * * Month: 1 to 12 (January to December)
 * * Day: 1 to 31
 * * Hour: 0 to 23
 * * Minute: 0 to 59
 * * Second: 0 to 59
 * * Centisecond: 0 to 99
 * * Weekday: 1 to 7 (Monday to Sunday)
 * 
 * If the backup battery is installed then the real-time clock keeps date and
 * time even if the Bricklet is not powered by a Brick.
 * 
 * The real-time clock handles leap year and inserts the 29th of February
 * accordingly. But leap seconds, time zones and daylight saving time are not
 * handled.
 */
int real_time_clock_v2_set_date_time(RealTimeClockV2 *real_time_clock_v2, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond, uint8_t weekday);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the current date (including weekday) and the current time of the
 * real-time clock with hundredths of a second resolution.
 * 
 * The timestamp represents the current date and the the current time of the
 * real-time clock converted to milliseconds.
 */
int real_time_clock_v2_get_date_time(RealTimeClockV2 *real_time_clock_v2, uint16_t *ret_year, uint8_t *ret_month, uint8_t *ret_day, uint8_t *ret_hour, uint8_t *ret_minute, uint8_t *ret_second, uint8_t *ret_centisecond, uint8_t *ret_weekday, int64_t *ret_timestamp);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the current date and the time of the real-time clock converted to
 * milliseconds. The timestamp has an effective resolution of hundredths of a
 * second.
 */
int real_time_clock_v2_get_timestamp(RealTimeClockV2 *real_time_clock_v2, int64_t *ret_timestamp);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the offset the real-time clock should compensate for in 2.17 ppm steps
 * between -277.76 ppm (-128) and +275.59 ppm (127).
 * 
 * The real-time clock time can deviate from the actual time due to the frequency
 * deviation of its 32.768 kHz crystal. Even without compensation (factory
 * default) the resulting time deviation should be at most ±20 ppm (±52.6
 * seconds per month).
 * 
 * This deviation can be calculated by comparing the same duration measured by the
 * real-time clock (``rtc_duration``) an accurate reference clock
 * (``ref_duration``).
 * 
 * For best results the configured offset should be set to 0 ppm first and then a
 * duration of at least 6 hours should be measured.
 * 
 * The new offset (``new_offset``) can be calculated from the currently configured
 * offset (``current_offset``) and the measured durations as follow::
 * 
 *   new_offset = current_offset - round(1000000 * (rtc_duration - ref_duration) / rtc_duration / 2.17)
 * 
 * If you want to calculate the offset, then we recommend using the calibration
 * dialog in Brick Viewer, instead of doing it manually.
 * 
 * The offset is saved in the EEPROM of the Bricklet and only needs to be
 * configured once.
 */
int real_time_clock_v2_set_offset(RealTimeClockV2 *real_time_clock_v2, int8_t offset);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the offset as set by {@link real_time_clock_v2_set_offset}.
 */
int real_time_clock_v2_get_offset(RealTimeClockV2 *real_time_clock_v2, int8_t *ret_offset);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the period in ms with which the {@link REAL_TIME_CLOCK_V2_CALLBACK_DATE_TIME} callback is triggered
 * periodically. A value of 0 turns the callback off.
 * 
 * The default value is 0.
 */
int real_time_clock_v2_set_date_time_callback_configuration(RealTimeClockV2 *real_time_clock_v2, uint32_t period);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the period as set by {@link real_time_clock_v2_set_date_time_callback_configuration}.
 */
int real_time_clock_v2_get_date_time_callback_configuration(RealTimeClockV2 *real_time_clock_v2, uint32_t *ret_period);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Configures a repeatable alarm. The {@link REAL_TIME_CLOCK_V2_CALLBACK_ALARM} callback is triggered if the
 * current date and time matches the configured alarm.
 * 
 * Setting a parameter to -1 means that it should be disabled and doesn't take part
 * in the match. Setting all parameters to -1 disables the alarm completely.
 * 
 * For example, to make the alarm trigger every day at 7:30 AM it can be
 * configured as (-1, -1, 7, 30, -1, -1, -1). The hour is set to match 7 and the
 * minute is set to match 30. The alarm is triggered if all enabled parameters
 * match.
 * 
 * The interval has a special role. It allows to make the alarm reconfigure itself.
 * This is useful if you need a repeated alarm that cannot be expressed by matching
 * the current date and time. For example, to make the alarm trigger every 23
 * seconds it can be configured as (-1, -1, -1, -1, -1, -1, 23). Internally the
 * Bricklet will take the current date and time, add 23 seconds to it and set the
 * result as its alarm. The first alarm will be triggered 23 seconds after the
 * call. Because the interval is not -1, the Bricklet will do the same again
 * internally, take the current date and time, add 23 seconds to it and set that
 * as its alarm. This results in a repeated alarm that triggers every 23 seconds.
 * 
 * The interval can also be used in combination with the other parameters. For
 * example, configuring the alarm as (-1, -1, 7, 30, -1, -1, 300) results in an
 * alarm that triggers every day at 7:30 AM and is then repeated every 5 minutes.
 */
int real_time_clock_v2_set_alarm(RealTimeClockV2 *real_time_clock_v2, int8_t month, int8_t day, int8_t hour, int8_t minute, int8_t second, int8_t weekday, int32_t interval);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the alarm configuration as set by {@link real_time_clock_v2_set_alarm}.
 */
int real_time_clock_v2_get_alarm(RealTimeClockV2 *real_time_clock_v2, int8_t *ret_month, int8_t *ret_day, int8_t *ret_hour, int8_t *ret_minute, int8_t *ret_second, int8_t *ret_weekday, int32_t *ret_interval);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the error count for the communication between Brick and Bricklet.
 * 
 * The errors are divided into
 * 
 * * ACK checksum errors,
 * * message checksum errors,
 * * framing errors and
 * * overflow errors.
 * 
 * The errors counts are for errors that occur on the Bricklet side. All
 * Bricks have a similar function that returns the errors on the Brick side.
 */
int real_time_clock_v2_get_spitfp_error_count(RealTimeClockV2 *real_time_clock_v2, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the bootloader mode and returns the status after the requested
 * mode change was instigated.
 * 
 * You can change from bootloader mode to firmware mode and vice versa. A change
 * from bootloader mode to firmware mode will only take place if the entry function,
 * device identifier and CRC are present and correct.
 * 
 * This function is used by Brick Viewer during flashing. It should not be
 * necessary to call it in a normal user program.
 */
int real_time_clock_v2_set_bootloader_mode(RealTimeClockV2 *real_time_clock_v2, uint8_t mode, uint8_t *ret_status);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the current bootloader mode, see {@link real_time_clock_v2_set_bootloader_mode}.
 */
int real_time_clock_v2_get_bootloader_mode(RealTimeClockV2 *real_time_clock_v2, uint8_t *ret_mode);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the firmware pointer for {@link real_time_clock_v2_write_firmware}. The pointer has
 * to be increased by chunks of size 64. The data is written to flash
 * every 4 chunks (which equals to one page of size 256).
 * 
 * This function is used by Brick Viewer during flashing. It should not be
 * necessary to call it in a normal user program.
 */
int real_time_clock_v2_set_write_firmware_pointer(RealTimeClockV2 *real_time_clock_v2, uint32_t pointer);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Writes 64 Bytes of firmware at the position as written by
 * {@link real_time_clock_v2_set_write_firmware_pointer} before. The firmware is written
 * to flash every 4 chunks.
 * 
 * You can only write firmware in bootloader mode.
 * 
 * This function is used by Brick Viewer during flashing. It should not be
 * necessary to call it in a normal user program.
 */
int real_time_clock_v2_write_firmware(RealTimeClockV2 *real_time_clock_v2, uint8_t data[64], uint8_t *ret_status);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Sets the status LED configuration. By default the LED shows
 * communication traffic between Brick and Bricklet, it flickers once
 * for every 10 received data packets.
 * 
 * You can also turn the LED permanently on/off or show a heartbeat.
 * 
 * If the Bricklet is in bootloader mode, the LED is will show heartbeat by default.
 */
int real_time_clock_v2_set_status_led_config(RealTimeClockV2 *real_time_clock_v2, uint8_t config);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the configuration as set by {@link real_time_clock_v2_set_status_led_config}
 */
int real_time_clock_v2_get_status_led_config(RealTimeClockV2 *real_time_clock_v2, uint8_t *ret_config);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the temperature in °C as measured inside the microcontroller. The
 * value returned is not the ambient temperature!
 * 
 * The temperature is only proportional to the real temperature and it has bad
 * accuracy. Practically it is only useful as an indicator for
 * temperature changes.
 */
int real_time_clock_v2_get_chip_temperature(RealTimeClockV2 *real_time_clock_v2, int16_t *ret_temperature);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Calling this function will reset the Bricklet. All configurations
 * will be lost.
 * 
 * After a reset you have to create new device objects,
 * calling functions on the existing ones will result in
 * undefined behavior!
 */
int real_time_clock_v2_reset(RealTimeClockV2 *real_time_clock_v2);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Writes a new UID into flash. If you want to set a new UID
 * you have to decode the Base58 encoded UID string into an
 * integer first.
 * 
 * We recommend that you use Brick Viewer to change the UID.
 */
int real_time_clock_v2_write_uid(RealTimeClockV2 *real_time_clock_v2, uint32_t uid);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the current UID as an integer. Encode as
 * Base58 to get the usual string version.
 */
int real_time_clock_v2_read_uid(RealTimeClockV2 *real_time_clock_v2, uint32_t *ret_uid);

/**
 * \ingroup BrickletRealTimeClockV2
 *
 * Returns the UID, the UID where the Bricklet is connected to,
 * the position, the hardware and firmware version as well as the
 * device identifier.
 * 
 * The position can be 'a', 'b', 'c' or 'd'.
 * 
 * The device identifier numbers can be found :ref:`here <device_identifier>`.
 * |device_identifier_constant|
 */
int real_time_clock_v2_get_identity(RealTimeClockV2 *real_time_clock_v2, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier);

#ifdef __cplusplus
}
#endif

#endif
