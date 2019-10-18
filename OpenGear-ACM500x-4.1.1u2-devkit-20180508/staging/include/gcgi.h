/* -*-mode:c; c-style:k&r; c-basic-offset:4; -*- */
/*
 * GCGI Library, implementing NCSA'a Common Gateway Interface and RFC2338.
 * Copyright (C) 2001-2002 Julian Catchen, julian@catchen.org
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __GCGI_H__
#define __GCGI_H__

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/*------ Enumerations ------*/
typedef enum {
    gcgiHttpCookie = 0,
    gcgiHttpReferer,
    gcgiAuthType,
    gcgiContentLength,
    gcgiContentType,
    gcgiGatewayInterface,
    gcgiPathInfo,
    gcgiPathTranslated,
    gcgiQueryString,
    gcgiRemoteAddr,
    gcgiRemoteHost,
    gcgiRemotePort,
    gcgiRemoteIdent,
    gcgiRemoteUser,
    gcgiRequestMethod,
    gcgiScriptName,
    gcgiServerName,
    gcgiServerPort,
    gcgiServerProtocol,
    gcgiServerSoftware,
    gcgiDontReadStdin,
    gcgiHttps,
} gcgiEnvVars;

/*------ Return Values ------*/
typedef enum {
    GCGIFATALERROR = -1,
    GCGISUCCESS,
    GCGIFIELDNOTFOUND,
    GCGIFIELDEMPTY,
    GCGITRUNCATED,
    GCGIBADDATA
} gcgiReturnType;


/*------ MIME types ------*/
typedef enum {
    text,
    image,
    audio,
    video,
    application,
    multipart,
    message,
    unknown
} MimeType;

typedef enum {
    sevenbit,
    eightbit,
    binary,
    quotedprintable,
    basesixtyfour,
    unknownEncoding
} MimeEncoding;

typedef enum {
    fixed,
    flowed
} MimeFormat;

typedef enum {
    inlined,
    attachment,
    formdata
} MimeDisposition;

typedef enum {
    contentType,
    contentEncoding,
    contentDisposition,
    contentDescription,
    mimeVersion,
    unknownHeader
} MimeHeader;

typedef enum {
    NONE = 0,
    COOKIE,
    DISPOSITION,
    LENGTH,
    LAST
} HTTPHeader;

/*------ CGI Functions ------*/

/* Initialization Functions */
gcgiReturnType  initCgi( void );
void            freeCgi( void );
gcgiReturnType  gcgiSetLimits(size_t fieldLimit, size_t queryLimit);

/* HTTP Header Functions */
gcgiReturnType  gcgiSendContentType(const char *mimeType, const char *name, const char *charset, HTTPHeader header);
gcgiReturnType  gcgiSendContentDisp(MimeDisposition disp, const char *filename, HTTPHeader header);
gcgiReturnType  gcgiSendContentLength(int length, HTTPHeader header);
gcgiReturnType  gcgiSendCacheControl(const char *cache, HTTPHeader header);
gcgiReturnType  gcgiSendLocation(const char *redirectURL);
gcgiReturnType  gcgiSendStatus(int status, const char *message);

/* Cookie Handling Functions */
gcgiReturnType  gcgiSendCookie(const char *name, const char *value, const char *path, const char *domain, const char *expires,
			       int secure, HTTPHeader header);
gcgiReturnType  gcgiFetchCookies(char ***cookies);
gcgiReturnType  gcgiParseCookie(const char *cookie, char **name, char **value);
gcgiReturnType  gcgiFreeCookies(char **cookies);

gcgiReturnType  gcgiSendEncryptedCookie(const char *name, const char *value, const char *path, const char *domain, const char *expires,
					int secure, const unsigned char *key, HTTPHeader header);
gcgiReturnType  gcgiParseEncryptedCookie(const char *cookie, const unsigned char *key, char **name, char **value);
gcgiReturnType  gcgiGenerateKey(unsigned char **key);
gcgiReturnType  gcgiWriteKeyToFile(const unsigned char *key, const char *path);
gcgiReturnType  gcgiReadKeyFromFile(const char *path, unsigned char **key);

/* Data Fetching Functions */
char           *gcgiFetchEnvVar(int env);
gcgiReturnType  gcgiFetchInteger(const char *field, int *ret, int defaultRet);
gcgiReturnType  gcgiFetchIntegerNext(const char *field, int *ret, int defaultRet);
gcgiReturnType  gcgiFetchDouble(const char *field, double *ret, double defaultRet);
gcgiReturnType  gcgiFetchDoubleNext(const char *field, double *ret, double defaultRet);
gcgiReturnType  gcgiFetchString(const char *field, char *ret, int max);
gcgiReturnType  gcgiFetchStringNext(const char *field, char *ret, int max);
gcgiReturnType  gcgiFetchStringNoNewLines(const char *field, char *ret, int max);
gcgiReturnType  gcgiFetchStringNoNewLinesNext(const char *field, char *ret, int max);
gcgiReturnType  gcgiFetchData(const char *field, char *ret, int max, MimeType *type,
			      char **subtype, MimeEncoding *encoding, char **filename, int *truncated);
gcgiReturnType  gcgiFetchDataNext(const char *field, char *ret, int max, MimeType *type,
				  char **subtype, MimeEncoding *encoding, char **filename, int *truncated);
gcgiReturnType  gcgiFieldLength(const char *field, int *ret);
gcgiReturnType  gcgiFieldLengthCur(const char *field, int *ret);
gcgiReturnType  gcgiFieldLengthNext(const char *field, int *ret);
gcgiReturnType  gcgiFieldSize(const char *field, int *ret);
gcgiReturnType  gcgiFieldSizeCur(const char *field, int *ret);
gcgiReturnType  gcgiFieldSizeNext(const char *field, int *ret);

gcgiReturnType  gcgiNumFormFields(int *ret);
gcgiReturnType  gcgiNumFields(const char *field, int *ret);

gcgiReturnType  gcgiFetchCheckbox(const char *field, int *ret);
gcgiReturnType  gcgiFetchMultipleCheckbox(const char *field, char **data, int size, int **ret);
gcgiReturnType  gcgiFetchMultipleString(const char *field, char ***data);
gcgiReturnType  gcgiFreeMultipleString(char **data);
gcgiReturnType  gcgiFetchSelectIndex(const char *field, char **data, int size, int *ret, int defaultVal);
gcgiReturnType  gcgiResetMultipleField(const char *field);
gcgiReturnType  gcgiLoadEnvVariables(const char *path);
gcgiReturnType  gcgiSaveEnvVariables(const char *path);
gcgiReturnType  gcgiDebug(const char *envVarsPath, const char *cgiQueryPath);

/* Encoding/Decoding Functions */
gcgiReturnType  gcgiDecodeBaseSixtyFourString(const char *text, char **decodedText, int *numBytes);
gcgiReturnType  gcgiEncodeBaseSixtyFourString(const char *text, int numBytes, char **encodedText);
gcgiReturnType  gcgiDecodeQuotedPrintableString(const char *text, char **decodedText, int *size);
gcgiReturnType  gcgiEncodeQuotedPrintableString(const char *text, char **encodedText, int *size);
gcgiReturnType  gcgiDecodeRfc2047String(const char *text, char **charset, char **decodedText);
gcgiReturnType  gcgiDecodeUrlEncodedString(const char *text, char **decodedText, int *size);
gcgiReturnType  gcgiEncodeUrlString(const char *text, char **encodedText, int *size);

/*------ Debugging Functions ------*/
gcgiReturnType  printQuery(FILE *stream);

/*------ Global Variables ------*/
extern FILE *gcgiOut;

#ifdef __cplusplus
}
#endif

#endif  /* __GCGI_H__ */
