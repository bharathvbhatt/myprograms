/*
 * Copyright 2005 Niels Provos <provos@citi.umich.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Niels Provos.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*	$OpenBSD: netdb.h,v 1.23 2004/12/20 22:35:32 millert Exp $	*/

/*
 * ++Copyright++ 1980, 1983, 1988, 1993
 * -
 * Copyright (c) 1980, 1983, 1988, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * -
 * Portions Copyright (c) 1993 by Digital Equipment Corporation.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies, and that
 * the name of Digital Equipment Corporation not be used in advertising or
 * publicity pertaining to distribution of the document or software without
 * specific, written prior permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND DIGITAL EQUIPMENT CORP. DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS.   IN NO EVENT SHALL DIGITAL EQUIPMENT
 * CORPORATION BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 * -
 * --Copyright--
 */

/*
 * Copyright (c) 1995, 1996, 1997, 1998, 1999 Craig Metz. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 *      @(#)netdb.h	8.1 (Berkeley) 6/2/93
 *	$From: netdb.h,v 8.7 1996/05/09 05:59:09 vixie Exp $
 */

#ifndef _DNSRES_H_
#define _DNSRES_H_

#include <sys/param.h>
#if (!defined(BSD)) || (BSD < 199306)
# include <sys/bitypes.h>
#endif
#include <sys/cdefs.h>

/*
 * Type values for resources and queries
 */
#define DNSRES_T_A	1	/* host address */
#define DNSRES_T_NS	2	/* authoritative server */
#define DNSRES_T_MD	3	/* mail destination */
#define DNSRES_T_MF	4	/* mail forwarder */
#define DNSRES_T_CNAME	5	/* canonical name */
#define DNSRES_T_SOA	6	/* start of authority zone */
#define DNSRES_T_MB	7	/* mailbox domain name */
#define DNSRES_T_MG	8	/* mail group member */
#define DNSRES_T_MR	9	/* mail rename name */
#define DNSRES_T_NULL	10	/* null resource record */
#define DNSRES_T_WKS	11	/* well known service */
#define DNSRES_T_PTR	12	/* domain name pointer */
#define DNSRES_T_HINFO	13	/* host information */
#define DNSRES_T_MINFO	14	/* mailbox information */
#define DNSRES_T_MX	15	/* mail routing information */
#define DNSRES_T_TXT	16	/* text strings */
#define DNSRES_T_RP	17	/* responsible person */
#define DNSRES_T_AFSDB	18	/* AFS cell database */
#define DNSRES_T_X25	19	/* X_25 calling address */
#define DNSRES_T_ISDN	20	/* ISDN calling address */
#define DNSRES_T_RT	21	/* router */
#define DNSRES_T_NSAP	22	/* NSAP address */
#define DNSRES_T_NSAP_PTR	23	/* reverse NSAP lookup (deprecated) */
#define DNSRES_T_SIG	24	/* security signature */
#define DNSRES_T_KEY	25	/* security key */
#define DNSRES_T_PX	26	/* X.400 mail mapping */
#define DNSRES_T_GPOS	27	/* geographical position (withdrawn) */
#define DNSRES_T_AAAA	28	/* IP6 Address */
#define DNSRES_T_LOC	29	/* Location Information */
#define DNSRES_T_NXT	30	/* Next Valid Name in Zone */
#define DNSRES_T_EID	31	/* Endpoint identifier */
#define DNSRES_T_NIMLOC	32	/* Nimrod locator */
#define DNSRES_T_SRV	33	/* Server selection */
#define DNSRES_T_ATMA	34	/* ATM Address */
#define DNSRES_T_NAPTR	35	/* Naming Authority PoinTeR */
#define DNSRES_T_KX	36	/* Key Exchanger */
#define DNSRES_T_CERT	37	/* CERT */
#define DNSRES_T_A6	38	/* A6 */
#define DNSRES_T_DNAME	39	/* DNAME */
#define DNSRES_T_SINK	40	/* SINK */
#define DNSRES_T_OPT	41	/* OPT pseudo-RR, RFC2671 */
#define DNSRES_T_APL	42	/* APL */
#define DNSRES_T_DS	43	/* Delegation Signer */
#define DNSRES_T_SSHFP	44	/* SSH Key Fingerprint */
#define DNSRES_T_RRSIG	46	/* RRSIG */
#define DNSRES_T_NSEC	47	/* NSEC */
#define DNSRES_T_DNSKEY	48	/* DNSKEY */
	/* non standard */
#define DNSRES_T_UINFO	100	/* user (finger) information */
#define DNSRES_T_UID	101	/* user ID */
#define DNSRES_T_GID	102	/* group ID */
#define DNSRES_T_UNSPEC	103	/* Unspecified format (binary data) */
	/* Query type values which do not appear in resource records */
#define DNSRES_T_TKEY	249	/* Transaction Key */
#define DNSRES_T_TSIG	250	/* Transaction Signature */
#define DNSRES_T_IXFR	251	/* incremental zone transfer */
#define DNSRES_T_AXFR	252	/* transfer zone of authority */
#define DNSRES_T_MAILB	253	/* transfer mailbox records */
#define DNSRES_T_MAILA	254	/* transfer mail agent records */
#define DNSRES_T_ANY	255	/* wildcard match */

/*
 * Internet nameserver port number
 */
#define DNSRES_NAMESERVER_PORT	53

/*
 * Currently defined opcodes
 */
#define DNSRES_QUERY		0x0	/* standard query */
#define DNSRES_IQUERY		0x1	/* inverse query */
#define DNSRES_STATUS		0x2	/* nameserver status query */
/*#define xxx			0x3*/	/* 0x3 reserved */
#define DNSRES_NS_NOTIFY_OP	0x4	/* notify secondary of SOA change */
/*
 * Currently defined response codes
 */
#define DNSRES_NOERROR		0		/* no error */
#define DNSRES_FORMERR		1		/* format error */
#define DNSRES_SERVFAIL		2		/* server failure */
#define DNSRES_NXDOMAIN		3		/* non existent domain */
#define DNSRES_NOTIMP		4		/* not implemented */
#define DNSRES_REFUSED		5		/* query refused */

/*
 * Values for class field
 */

#define DNSRES_C_IN		1	/* the arpa internet */
#define DNSRES_C_CHAOS		3	/* for chaos net (MIT) */
#define DNSRES_C_HS		4	/* for Hesiod name server (MIT)(XXX) */
	/* Query class values which do not appear in resource records */
#define DNSRES_C_ANY		255	/* wildcard match */

/*
 * EDNS0 Z-field extended flags
 */
#define DNSRES_MESSAGEEXTFLAG_DO   0x8000U

#define	DNSRES_PATH_HEQUIV	"/etc/hosts.equiv"
#define	DNSRES_PATH_HOSTS	"/etc/hosts"
#define	DNSRES_PATH_NETWORKS	"/etc/networks"
#define	DNSRES_PATH_PROTOCOLS	"/etc/protocols"
#define	DNSRES_PATH_SERVICES	"/etc/services"

/*
 * Structures returned by network data base library.  All addresses are
 * supplied in host order, and returned in network order (suitable for
 * use in system calls).
 */
struct	dnsres_hostent {
	char	*h_name;	/* official name of host */
	char	**h_aliases;	/* alias list */
	int	h_addrtype;	/* host address type */
	int	h_length;	/* length of address */
	char	**h_addr_list;	/* list of addresses from name server */
};

/*
 * Assumption here is that a network number
 * fits in an in_addr_t -- probably a poor one.
 */
struct	dnsres_netent {
	char		*n_name;	/* official name of net */
	char		**n_aliases;	/* alias list */
	int		n_addrtype;	/* net address type */
	in_addr_t	n_net;		/* network # */
};

struct	dnsres_servent {
	char	*s_name;	/* official service name */
	char	**s_aliases;	/* alias list */
	int	s_port;		/* port # */
	char	*s_proto;	/* protocol to use */
};

struct	dnsres_protoent {
	char	*p_name;	/* official protocol name */
	char	**p_aliases;	/* alias list */
	int	p_proto;	/* protocol # */
};

/*
 * Error return codes from gethostbyname() and gethostbyaddr()
 * (left in extern int dr_errno).
 */

#define	DNSRES_NETDB_INTERNAL	-1	/* see errno */
#define	DNSRES_NETDB_SUCCESS	0	/* no problem */
#define	DNSRES_HOST_NOT_FOUND	1 /* Authoritative Answer Host not found */
#define	DNSRES_TRY_AGAIN	2 /* Non-Authoritive Host not found, or SERVERFAIL */
#define	DNSRES_NO_RECOVERY	3 /* Non recoverable errors, FORMERR, REFUSED, NOTIMP */
#define	DNSRES_NO_DATA		4 /* Valid name, no data record of requested type */
#define	DNSRES_NO_ADDRESS	DNSRES_NO_DATA	/* no address, look for MX record */

/* Values for getaddrinfo() and getnameinfo() */
#define DNSRES_AI_PASSIVE	1	/* socket address is intended for bind() */
#define DNSRES_AI_CANONNAME	2	/* request for canonical name */
#define DNSRES_AI_NUMERICHOST	4	/* don't ever try hostname lookup */
#define DNSRES_AI_EXT		8	/* enable non-portable extensions */
#define DNSRES_AI_NUMERICSERV	16	/* don't ever try servname lookup */
/* valid flags for addrinfo */
#define DNSRES_AI_MASK \
    (DNSRES_AI_PASSIVE | DNSRES_AI_CANONNAME | \
     DNSRES_AI_NUMERICHOST | DNSRES_AI_NUMERICSERV)

#define DNSRES_NI_NUMERICHOST	1	/* return the host address, not the name */
#define DNSRES_NI_NUMERICSERV	2	/* return the service address, not the name */
#define DNSRES_NI_NOFQDN	4	/* return a short name if in the local domain */
#define DNSRES_NI_NAMEREQD	8	/* fail if either host or service name is unknown */
#define DNSRES_NI_DGRAM	16	/* look up datagram service instead of stream */

#define DNSRES_NI_MAXHOST	MAXHOSTNAMELEN	/* max host name returned by getnameinfo */
#define DNSRES_NI_MAXSERV	32	/* max serv. name length returned by getnameinfo */

/*
 * Scope delimit character (KAME hack)
 */
#define DNSRES_SCOPE_DELIMITER '%'

#define DNSRES_EAI_BADFLAGS	-1	/* invalid value for ai_flags */
#define DNSRES_EAI_NONAME	-2	/* name or service is not known */
#define DNSRES_EAI_AGAIN	-3	/* temporary failure in name resolution */
#define DNSRES_EAI_FAIL	-4	/* non-recoverable failure in name resolution */
#define DNSRES_EAI_NODATA	-5	/* no address associated with name */
#define DNSRES_EAI_FAMILY	-6	/* ai_family not supported */
#define DNSRES_EAI_SOCKTYPE	-7	/* ai_socktype not supported */
#define DNSRES_EAI_SERVICE	-8	/* service not supported for ai_socktype */
#define DNSRES_EAI_ADDRFAMILY	-9	/* address family for name not supported */
#define DNSRES_EAI_MEMORY	-10	/* memory allocation failure */
#define DNSRES_EAI_SYSTEM	-11	/* system error (code indicated in errno) */
#define DNSRES_EAI_BADHINTS	-12	/* invalid value for hints */
#define DNSRES_EAI_PROTOCOL	-13	/* resolved protocol is unknown */

/*
 * Flags for getrrsetbyname()
 */
#define DNSRES_RRSET_VALIDATED		1

/*
 * Return codes for getrrsetbyname()
 */
#define DNSRES_ERRSET_SUCCESS		0
#define DNSRES_ERRSET_NOMEMORY		1
#define DNSRES_ERRSET_FAIL		2
#define DNSRES_ERRSET_INVAL		3
#define DNSRES_ERRSET_NONAME		4
#define DNSRES_ERRSET_NODATA		5

/*
 * Structures used by getrrsetbyname() and freerrset()
 */
struct dnsres_rdatainfo {
	unsigned int		rdi_length;	/* length of data */
	unsigned char		*rdi_data;	/* record data */
};

struct dnsres_rrsetinfo {
	unsigned int		rri_flags;	/* RRSET_VALIDATED ... */
	unsigned int		rri_rdclass;	/* class number */
	unsigned int		rri_rdtype;	/* RR type number */
	unsigned int		rri_ttl;	/* time to live */
	unsigned int		rri_nrdatas;	/* size of rdatas array */
	unsigned int		rri_nsigs;	/* size of sigs array */
	char			*rri_name;	/* canonical name */
	struct dnsres_rdatainfo	*rri_rdatas;	/* individual records */
	struct dnsres_rdatainfo	*rri_sigs;	/* individual signatures */
};

#ifndef POSIX_SOURCE
struct dnsres_servent_data {
	void *fp;
	char **aliases;
	int maxaliases;
	int stayopen;
	char *line;
};

struct dnsres_protoent_data {
	void *fp;
	char **aliases;
	int maxaliases;
	int stayopen;
	char *line;
};
#endif

__BEGIN_DECLS
struct addrinfo;
struct dnsres;
struct dnsres_cbstate;
struct dnsres_servent_state;
struct dnsres_hostent_state;
void		dnsres_endhostent(void);
void		dnsres_endnetent(void);
void		dnsres_endprotoent(void);
void		dnsres_endservent(struct dnsres_servent_state *);
void		*dnsres_gethostbyaddr(struct dnsres *_resp,
		    const char *addr, int len, int af,
		    void (*cb)(struct dnsres_hostent *, int, void *),
		    void *arg);
void		*dnsres_gethostbyname(struct dnsres *_resp, const char *name,
		    void (*cb)(struct dnsres_hostent *, int, void *),
		    void *arg);
void		*dnsres_gethostbyname2(struct dnsres *_resp, const char *name,
		    int af, void (*cb)(struct dnsres_hostent *, int, void *),
		    void *arg);
struct dnsres_hostent
		*dnsres_gethostent(struct dnsres *_resp,
		    struct dnsres_cbstate *state);
struct dnsres_netent
		*dnsres_getnetbyaddr(in_addr_t, int);
struct dnsres_netent
		*dnsres_getnetbyname(const char *);
struct dnsres_netent
		*dnsres_getnetent(void);
struct dnsres_protoent
		*dnsres_getprotobyname(const char *);
struct dnsres_protoent
		*dnsres_getprotobynumber(int);
struct dnsres_protoent
		*dnsres_getprotoent(void);
struct dnsres_servent
		*dnsres_getservbyname(struct dnsres_servent_state *,
		    const char *name, const char *proto,
		    struct dnsres_servent *se, char *buf, int buflen);
struct dnsres_servent
		*dnsres_getservbyport(int, const char *);
struct dnsres_servent
		*dnsres_getservent(struct dnsres_servent_state *);
void		dnsres_herror(const char *);
const char	*dnsres_hstrerror(int);
void		dnsres_sethostent(int);
void		dnsres_setnetent(int);
void		dnsres_setprotoent(int);
void		dnsres_setservent(struct dnsres_servent_state *, int);

void		dnsres_getaddrinfo(struct dnsres *, const char *, const char *,
		    const struct addrinfo *,
		    void (*)(struct addrinfo *, int, void *),
		    void *);
const char	*dnsres_gai_strerror(int);
int		dnsres_net_addrcmp(struct sockaddr *, struct sockaddr *);
int		dnsres_getrrsetbyname(const char *, unsigned int, unsigned int, unsigned int, struct dnsres_rrsetinfo **);
void		dnsres_freerrset(struct dnsres_rrsetinfo *);
__END_DECLS

/*
 * Global defines and variables for resolver stub.
 */

#define	MAXNS			3	/* max # name servers we'll track */
#define	MAXDFLSRCH		3	/* # default domain levels to try */
#define	MAXDNSRCH		6	/* max # domains in search path */
#define	LOCALDOMAINPARTS	2	/* min levels in name that is "local" */
#define MAXDNSLUS		4	/* max # of host lookup types */

#define	RES_TIMEOUT		5	/* min. seconds between retries */
#define	MAXRESOLVSORT		10	/* number of net to sort on */
#define	RES_MAXNDOTS		15	/* should reflect bit field size */

#define	MAXALIASES		35

struct dnsres_servent_state {
	FILE *servf;
	char line[BUFSIZ+1];
	struct dnsres_servent serv;
	char *serv_aliases[MAXALIASES];
	int stayopen;
};

struct dnsres_hostent_state {
	FILE *hostf;
	int stayopen;
};

struct dnsres {
	int	retrans;	 	/* retransmission time interval */
	int	retry;			/* number of times to retransmit */
	unsigned long	options;	/* option flags - see below. */
	int	nscount;		/* number of name servers */
	struct sockaddr_in
		nsaddr_list[MAXNS];	/* address of name server */
#define	nsaddr	nsaddr_list[0]		/* for backward compatibility */
	unsigned short	id;		/* current message id */
	char	*dnsrch[MAXDNSRCH+1];	/* components of domain to search */
	char	defdname[256];		/* default domain (deprecated) */
	unsigned long	pfcode;		/* RES_PRF_ flags - see below. */
	unsigned ndots:4;		/* threshold for initial abs. query */
	unsigned nsort:4;		/* number of elements in sort_list[] */
	char	unused[3];
	struct {
		struct in_addr	addr;
		u_int32_t	mask;
	} sort_list[MAXRESOLVSORT];
	char    lookups[MAXDNSLUS];

	int dr_errno;				/* keep track of errors */
/*
 * replacement of dnsres, separated to keep binary compatibility.
 * XXX - niels - included until I grok the code better.
 */
	struct dnsres_ext {
		struct sockaddr_storage nsaddr_list[MAXNS];
		struct {
			int	af;	/* address family for addr, mask */
			union {
				struct in_addr ina;
				struct in6_addr in6a;
			} addr, mask;
		} sort_list[MAXRESOLVSORT];
	} ext;

	struct dnsres_hostent_state hostent_state;
	struct dnsres_servent_state servent_state;
};

int dnsres_init(struct dnsres *_resp);
void dnsres_cancel_lookup(void *handle);

#endif /* _DNSRES_H_ */
