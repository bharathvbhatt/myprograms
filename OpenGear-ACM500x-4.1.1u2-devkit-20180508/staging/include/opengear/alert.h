#ifndef __H_LIB_OPENGEAR_ALERT_H__
#define __H_LIB_OPENGEAR_ALERT_H__

#include <scew/scew.h>

#define CMDSOCKET_PATH "/var/run/alert"

typedef struct alertRequest {
	scew_tree * request;
	scew_element * root;
	scew_element * command;
	scew_element * environment;
} alert_request_t;

/**
 * Create an alert request
 * @return A new request struct, or NULL on failure
 */
alert_request_t * createAlertRequest();

/**
 * Delete an alert request
 * @param request Item to be removed
 */
void freeAlertRequest( alert_request_t * request );

/**
 * Set the command to be run for the alert
 * @param request The request structure
 * @param cmd The command to run
 */
void setAlertCommand( alert_request_t * request, char * cmd );

/**
 * Set an environment variable for the the alert
 * @param request The request structure
 * @param env The environment variable name
 * @param val The value of theenvironment variable
 */
void addAlertEnvironment( alert_request_t * request, char * env, char * val );

/**
 * Trigger sending the alert
 * @param request The request to send
 * @return 0 for success, 1 for failure.  Success does not guarantee the
 * alert was sent, just that it was queued.
 */
int sendAlertRequest( alert_request_t * request );

/**
 * Debug function to print the alert XML tree
 * @param request The request structure
 */
void printAlertRequest(alert_request_t * request);

#endif
