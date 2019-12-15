/*
 * Copyright (C) 2012-2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//+bug241393,shihaikuo.wt,add,20170512,add for logcatkmsg tool

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <zlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include "pwd.h"
#include "errno.h"
#include "wt_compress.h"

#define HEADERNUM 512
#define CHECKSUMAPPEND 256
#define NAMELEN 100
#define FILEMODE 8
#define UID 8
#define GID 8
#define FILESIZE 12
#define UNIXTIME 12
#define CHECKSUM 8

/* define the struct of tar file header */
typedef struct tag_header {
    char name[NAMELEN]; /* 0 */
    char mode[FILEMODE]; /* 100 */
    char uid[UID]; /* 108 */
    char gid[GID]; /* 116 */
    char size[FILESIZE]; /* 124 */
    char mtime[UNIXTIME]; /* 136 */
    char chksum[CHECKSUM]; /* 148 */
    char typeflag; /*156 */
    char linkname[100]; /* 157 */
    char magic[6]; /* 257 */
    char version[2]; /* 263 */
    char uname[32]; /* 265 */
    char gname[32]; /* 297 */
    char devmajor[8]; /* 329 */
    char devminor[8]; /* 337 */
    char prefix[155]; /* 345 */
    char padding[12]; /*500*/
    /* 512*/
}header;

/* tar file header mode permissions data */
static const char file_mode[ ] = { 0x31, 0x30, 0x30, 0x36, 0x36, 0x36, 0x20, 0 };

/* tar file header UID and GID data */
static const char IDDATA[] = {
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x30, 0x20, 0x00,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x30, 0x20, 0x00
};

/* write the tar file header to header struct */
void wt_write_compress_header(header *hd, const char *file_name, long file_size)
{
    int o =0, p = 0, q = 0, r = 0, s =0;
    int wt_name_len  = 0;
    int i = 0, j = 0;
    char* index = (char *)hd;
    char temp_buf[FILESIZE];
    const char *wt_file_index;

    memset(temp_buf, 0, FILESIZE);
    wt_file_index = file_name;
    wt_name_len =  strlen(file_name);
    if(wt_name_len > NAMELEN) {
        wt_name_len = NAMELEN;
    }

    for (i=0; i<wt_name_len; i++ ) {
        index[i] = wt_file_index[i];
        o += index[i];
    }
    j += NAMELEN;

    for(i=0; i<FILEMODE; i++) {
        index[j+i] = file_mode[i];
        p += file_mode[i];
    }
    j += FILEMODE;

    for(i=0; i<UID+GID; i++) {
    index[j+i] = IDDATA[i];
    q += IDDATA[i];
    }
    j += (UID+GID);

    snprintf(temp_buf, FILESIZE, "%o", (unsigned int)file_size);
    for(i=0; i<FILESIZE; i++) {
        index[j+i] = temp_buf[i];
    r += temp_buf[i];
    }
    j += (FILESIZE + UNIXTIME);
    o += (p +q +r + CHECKSUMAPPEND);

    memset(temp_buf, 0, CHECKSUM);
    snprintf(temp_buf, CHECKSUM, "%o", o);
    for(i=0; i<CHECKSUM; i++) {
        index[j+i] = temp_buf[i];
    }
}

void wt_write_compress_content(gzFile fd, const char* filename, long file_size)
{
    char temp_buf[HEADERNUM]= { 0 };
    FILE *wt_in = fopen(filename, "r");
    if(NULL == wt_in) {
        return;
    }
    while(file_size > 0) {
        memset(temp_buf, 0, HEADERNUM);
        fread(temp_buf, HEADERNUM, 1, wt_in);
        gzwrite(fd, temp_buf, HEADERNUM);
        file_size -= HEADERNUM;
    }
    fclose(wt_in);
}

static void wt_compress_dir_recursive(gzFile out, const char* dirname, const char * parent)
{
    DIR *dp;
    struct dirent *dirp;
    char wt_target_file_buf[MAX_FILE_NAME_LEN] = {0};
    char *ptr =NULL;

    if((dp=opendir(dirname)) == NULL) {
        return;
    }
    memset(wt_target_file_buf, 0, MAX_FILE_NAME_LEN);
    strcpy(wt_target_file_buf, dirname);
    ptr = wt_target_file_buf + strlen(wt_target_file_buf);
    if((*ptr-1) != '/') {
        *ptr ++= '/';
        *ptr = 0;
    }

    while((dirp = readdir(dp)) != NULL) {
        if(strcmp(dirp->d_name, ".") == 0 ||strcmp(dirp->d_name, "..") == 0) {
            continue;
        }
        strcpy(ptr, dirp->d_name);

        struct stat tmp_buf;
        if(lstat(wt_target_file_buf, &tmp_buf) < 0) {
            continue;
        }

        if(S_ISREG(tmp_buf.st_mode)) {
            header t;
            memset(&t, 0, sizeof(t));
            if(parent) {
                char szfilename[NAMELEN] = {0};
                snprintf(szfilename, NAMELEN, "%s/%s", parent, dirp->d_name);
                wt_write_compress_header(&t, szfilename, tmp_buf.st_size);
            }else {
                wt_write_compress_header(&t, dirp->d_name, tmp_buf.st_size);
            }

            gzwrite(out, &t, sizeof(header));
            wt_write_compress_content(out, wt_target_file_buf, tmp_buf.st_size);
        }else if(S_ISDIR(tmp_buf.st_mode)) {
            char CpPath[NAMELEN] = {0};
            strncpy(CpPath, dirp->d_name, NAMELEN-1);
            if(parent) {
                snprintf(CpPath, NAMELEN-1, "%s/%s", parent, dirp->d_name);
                wt_compress_dir_recursive(out, wt_target_file_buf, CpPath);
            }else {
                wt_compress_dir_recursive(out, wt_target_file_buf, NULL);
            }
        }
    }
    closedir(dp);
}

/* compress  files to output file */
void wt_compress_zip_files(int counts, const char** input, char* output, int keep_status)
{
    gzFile out = NULL;
    if(output) {
        out = gzopen(output, "w");
    }
    if( !out ) {
        fprintf (stderr, "%s gzopen failed ....\n", __func__);
        return;
    }

    struct passwd *p = getpwnam("system");
    if(p) {
        chown(output,  p->pw_uid, p->pw_gid);
    }
    const char **index = input;
    header t;
    int i;

    for (i=0; i < counts; i++) {
        if(index == NULL) {
            ++index;
            fprintf (stderr, "%s index ==NULL continue ....\n", __func__);
            continue;
        }
        struct stat buf;
        if(lstat(*index, &buf) < 0) {
            index ++;
            fprintf (stderr, "%s if(lstat(*index, &buf) < 0, index =%s, continue ..., errno: %s \n", __func__, *index, strerror(errno));
            continue;
        }

        if(S_ISREG(buf.st_mode)) {
            memset(&t, 0, sizeof(t));
            wt_write_compress_header(&t, strrchr(*index, '/')+1, buf.st_size);
            gzwrite(out, &t, sizeof(header));
            wt_write_compress_content(out, *index, buf.st_size);
        }else if(S_ISDIR(buf.st_mode)) {
            if(keep_status) {
                char *p  = NULL;
                char szTmp[NAMELEN]  = {0};
                strncpy(szTmp, *index, NAMELEN-1);
                p = szTmp + strlen(szTmp);
                if(*(--p) == '/') {
                    *p ='\0';
                }
                p = strrchr(szTmp, '/');
                wt_compress_dir_recursive(out, *index, p+1);
            }else {
                wt_compress_dir_recursive(out, *index, NULL);
            }
        }
        ++index;
    }
    memset(&t, 0, sizeof(t));
    gzwrite(out, &t, sizeof(header));
    gzclose(out);
}
//-bug241393,shihaikuo.wt,add,20170512,add for logcatkmsg tool
