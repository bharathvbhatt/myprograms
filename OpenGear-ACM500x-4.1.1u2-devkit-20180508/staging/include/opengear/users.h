#ifndef _OPENGEAR_USERS_H_
#define _OPENGEAR_USERS_H_

#include <sys/types.h>
#include <grp.h>
#include <pwd.h>

#include <stdbool.h>

#include <scew/types.h>

#include <opengear/xmldb.h>
#include <opengear/og_config.h>

__BEGIN_DECLS

/**
 * Determine if the given user is a member of the given group.
 * @param pw A passwd account entry.
 * @param gr The group entry user may belong to.
 * @return true if user is a primary or secondary member otherwise false.
 */
bool opengear_users_ispwingroup(struct passwd *pw, struct group *gr);

/**
 * Get the allowed ports for a user
 * @param db A handle to the configuration database
 * @param ports A zero-indexed array to record the port permissions
 * @param numports The number of indices in ports
 * @param uid The User-ID of the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 */
void opengear_users_getallowedports(xmldb_t *db, bool ports[], int numports, uid_t uid, gid_t gid);

#ifdef HAVE_SERIAL_CONCENTRATOR
/**
 * Get the allowed ports for a user from a remote console server
 * @param db A handle to the configuration database
 * @param remote_name The name of the remote console server
 * @param ports A zero-indexed array to record the port permissions
 * @param numports The number of indices in ports
 * @param uid The User-ID of the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 */
void opengear_users_getallowedremoteports(xmldb_t *db, const char *remote_name, bool ports[], int numports, uid_t uid, gid_t gid);
#endif

#ifdef HAVE_SERIAL_CONCENTRATOR
/**
 * Get the allowed remote consoles for a user 
 * @param db A handle to the configuration database
 * @param ports A zero-indexed array to record the remotes permissions
 * @param numports The number of indices in remoteconsoles
 * @param uid The User-ID of the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @param remote_name The name of the remote console server
 */
void
opengear_users_getallowedremoteconsoles(xmldb_t *db, bool remoteconsoles[], int numremoteconsoles, uid_t uid, gid_t gid);
#endif

#ifdef HAVE_SERIAL_CONCENTRATOR
/**
 * Determine if the given user can access the specified remote console server.
 * @param db A handle to the configuration database.
 * @param port The serial port in question (1 - NPORTS)
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isremoteconsolellowed(xmldb_t *db, const char *remote_name, uid_t uid, gid_t gid);
bool opengear_groups_isremoteconsoleallowed(xmldb_t *db, const char *rmote_name, gid_t gid);
bool opengear_users_isremoteconsolevisible(xmldb_t *db, const char *remote_name, uid_t uid, gid_t gid);
bool opengear_groups_isremoteconsolevisible(xmldb_t *db, const char *remote_name, gid_t gid);
#endif

/**
 * Determine if the given user can access the specified port.
 * @param db A handle to the configuration database.
 * @param port The serial port in question (1 - NPORTS)
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isportallowed(xmldb_t *db, int port, uid_t uid, gid_t gid);
bool opengear_groups_isportallowed(xmldb_t *db, int port, gid_t gid);
bool opengear_users_isportvisible(xmldb_t *db, int port, uid_t uid, gid_t gid);
bool opengear_groups_isportvisible(xmldb_t *db, int port, gid_t gid);

/**
 * Determine if the given user can access the specified host.
 * @param db A handle to the configuration database.
 * @param address The unique host address
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_ishostallowed(
	xmldb_t *db, const char *address, uid_t uid, gid_t gid);
bool opengear_groups_ishostallowed(
	xmldb_t *db, const char *address, gid_t gid);
bool opengear_users_ishostvisible(
	xmldb_t *db, const char *address, uid_t uid, gid_t gid);
bool opengear_groups_ishostvisible(
	xmldb_t *db, const char *address, gid_t gid);

/**
 * Determine if the given user can access the specified RPC outlet.
 * @param db A handle to the configuration database.
 * @param configpath The unique outlet config path.
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isoutletallowed(
	xmldb_t *db, const char *configpath, uid_t uid, gid_t gid);
bool opengear_groups_isoutletallowed(
	xmldb_t *db, const char *configpath, gid_t gid);
bool opengear_users_isoutletvisible(
	xmldb_t *db, const char *configpath, uid_t uid, gid_t gid);
bool opengear_groups_isoutletvisible(
	xmldb_t *db, const char *configpath, gid_t gid);

/**
 * Determine if the given user can access the specified network RPC outlet.
 * @param db A handle to the configuration database.
 * @param address The RPC network address.
 * @param outletnum Index of the outlet.
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isoutletallowedbyaddress(
	xmldb_t *db, const char *address, unsigned int outletnum,
	uid_t uid, gid_t gid);
bool opengear_users_isoutletvisiblebyaddress(
	xmldb_t *db, const char *address, unsigned int outletnum,
	uid_t uid, gid_t gid);

/**
 * Determine if the given user can access the specified serial RPC outlet.
 * @param db A handle to the configuration database.
 * @param device RPC serial port device node.
 * @param outletnum Index of the outlet.
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isoutletallowedbydevice(
	xmldb_t *db, const char *device, unsigned int outletnum,
	uid_t uid, gid_t gid);
bool opengear_users_isoutletvisiblebydevice(
	xmldb_t *db, const char *device, unsigned int outletnum,
	uid_t uid, gid_t gid);

/**
 * Determine if the given user can access the specified serial RPC outlet.
 * @param db A handle to the configuration database.
 * @param portnum Index of the serial port.
 * @param outletnum Index of the outlet.
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isoutletallowedbyportnum(
	xmldb_t *db, unsigned int portnum, unsigned int outletnum,
	uid_t uid, gid_t gid);
bool opengear_users_isoutletvisiblebyportnum(
	xmldb_t *db, unsigned int portnum, unsigned int outletnum,
	uid_t uid, gid_t gid);

/**
 * Determine if the given user can access one or more outlets on
 * the specified RPC.
 * @param db A handle to the configuration database.
 * @param configpath The unique RPC config path.
 * @param uid The User-ID if the user attempting access.
 * @param gid The Group-ID of the user attempting access.
 * @return true if permitted otherwise false.
 */
bool opengear_users_isrpcallowed(
	xmldb_t *db, const char *configpath, uid_t uid, gid_t gid);
bool opengear_groups_isrpcallowed(
	xmldb_t *db, const char *configpath, gid_t gid);
bool opengear_users_isrpcvisible(
	xmldb_t *db, const char *configpath, uid_t uid, gid_t gid);
bool opengear_groups_isrpcvisible(
	xmldb_t *db, const char *configpath, gid_t gid);

/**
 * Determine the next available uid
 * @return the expected next value for uid
 */
int getNextUid();

/**
 * Get a list of local groups that the user is a member of.
 * This is primarily used by pam_adduser. Caller must free the array, and the contents.
 */
char **opengear_users_getlocalgroups(xmldb_t *db, const char *username, size_t *num_groups);

__END_DECLS

#endif /* _OPENGEAR_USERS_H_ */
