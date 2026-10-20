/* Ogre 2 hardcodes ~/.gz/rendering/ogre2.log. Redirect only that file for
 * this project's child process; leave the user's HOME and other files alone. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *redirect(const char *path) {
    const char *suffix="/.gz/rendering/ogre2.log";
    const char *dest=getenv("LIVO_OGRE_LOG_PATH");
    if(path && dest && strlen(path)>=strlen(suffix) &&
       strcmp(path+strlen(path)-strlen(suffix),suffix)==0) return dest;
    return path;
}
FILE *fopen(const char *path,const char *mode) {
    FILE *(*fn)(const char *,const char *)=dlsym(RTLD_NEXT,"fopen");
    return fn(redirect(path),mode);
}
FILE *fopen64(const char *path,const char *mode) {
    FILE *(*fn)(const char *,const char *)=dlsym(RTLD_NEXT,"fopen64");
    return fn(redirect(path),mode);
}
int open(const char *path,int flags,...) {
    mode_t mode=0;
    if((flags&O_CREAT) || ((flags&O_TMPFILE)==O_TMPFILE)) {
        va_list args; va_start(args,flags); mode=va_arg(args,int); va_end(args);
    }
    int (*fn)(const char *,int,...)=dlsym(RTLD_NEXT,"open");
    return fn(redirect(path),flags,mode);
}
int open64(const char *path,int flags,...) {
    mode_t mode=0;
    if((flags&O_CREAT) || ((flags&O_TMPFILE)==O_TMPFILE)) {
        va_list args; va_start(args,flags); mode=va_arg(args,int); va_end(args);
    }
    int (*fn)(const char *,int,...)=dlsym(RTLD_NEXT,"open64");
    return fn(redirect(path),flags,mode);
}
