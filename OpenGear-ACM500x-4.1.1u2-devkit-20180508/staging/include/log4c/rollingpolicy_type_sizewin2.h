
/*
 * rollingpolicy_type_sizewin2.h
 *
 * See the COPYING file for the terms of usage and distribution.
*/

#ifndef log4c_policy_type_sizewin2_h
#define log4c_policy_type_sizewin2_h

/**
 * @file rollingpolicy_type_sizewin2.h
 *
 * @brief Log4c rolling file size-win interface that appends to existing files.
 * This enhances the classic size-window rollover policy:
 * this triggers rollover when files reach a maximum size or number of lines.
 * The first file in the list is
 * always the current file; when a rollover event occurs files are shifted up
 * by one position in the list--if the number of files in the list has already
 * reached the max then the oldest file is rotated out of the window.
 *
 * If the max file size is set to zero, this means 'no-limit' unless the max
 * file lines are set in which case the file rolls when the given amount of
 * new-lines are logged. If both max file size and max file lines are not set
 * then behavior reverts to 'no-limit'.
 *
 * The default parameters for the size-win policy are 5 files of maximum
 * size of 20kilobytes each.  These parameters may be changed using the
 * appropriate setter functions.
 */

#include <log4c/defs.h>
#include <log4c/rollingpolicy.h>

__LOG4C_BEGIN_DECLS

LOG4C_API const log4c_rollingpolicy_type_t log4c_rollingpolicy_type_sizewin2;
/**
 * log4c size-win rolling policy type
*/
typedef struct __sizewin2_udata rollingpolicy_sizewin2_udata_t;

#define ROLLINGPOLICY_SIZE2_DEFAULT_MAX_FILE_SIZE 1024*20
#define ROLLINGPOLICY_SIZE2_DEFAULT_MAX_FILE_LINES 0
#define ROLLINGPOLICY_SIZE2_DEFAULT_MAX_NUM_FILES 5

/**
 * Get a new size-win rolling policy
 * @return a new size-win rolling policy, otherwise NULL.
 */
LOG4C_API rollingpolicy_sizewin2_udata_t *sizewin2_make_udata(void);

/**
 * Set the maximum file size in this rolling policy configuration.
 * @param swup the size-win configuration object.
 * @param max_size the approximate maximum size any logging file will
 * attain.
 * If you set zero then it means 'no-limit' and so only one file
 * of unlimited size will be used for logging.
 * @return zero if successful, non-zero otherwise.
 */
LOG4C_API int sizewin2_udata_set_file_maxsize(
                              rollingpolicy_sizewin2_udata_t * swup,
			      long max_size);
/**
 * Set the maximum file lines in this rolling policy configuration.
 * @param swup the size-win configuration object.
 * @param max_lines the approximate maximum lines any logging file will
 * attain.
 * If you set zero then it means 'no-limit' and so only one file
 * of unlimited lines will be used for logging unless max_size is set.
 * @return zero if successful, non-zero otherwise.
 */
LOG4C_API int sizewin2_udata_set_file_maxlines(
                              rollingpolicy_sizewin2_udata_t * swup,
			      long max_lines);

/**
 * Set the maximum number of filesin this rolling policy configuration.
 * @param swup the size-win configuration object.
 * @param max_num the maximum number of files in the list.
 * @return zero if successful, non-zero otherwise.
 */
LOG4C_API int sizewin2_udata_set_max_num_files(
                              rollingpolicy_sizewin2_udata_t * swup,
	                      long max_num);

/**
 * Set the rolling file appender in this rolling policy configuration.
 * @param swup the size-win configuration object.
 * @param app the rolling file appender to set.
 * @return zero if successful, non-zero otherwise.
*/
LOG4C_API int sizewin2_udata_set_appender(
                              rollingpolicy_sizewin2_udata_t * swup,
			      log4c_appender_t* app);

__LOG4C_END_DECLS


#endif
