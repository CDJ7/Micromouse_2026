/*
 * syscalls.c - the few C-library hooks the firmware needs.
 *
 * newlib's vsnprintf (used by hal_log) drags in file-system hooks that a bare
 * microcontroller does not have. These do-nothing versions satisfy the linker
 * without the "not implemented" warnings. Text goes out through hal_log(),
 * not through stdout.
 */
#include <errno.h>
#include <sys/stat.h>

int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }
int _read(int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; return 0; }
int _write(int fd, const char *buf, int len) { (void)fd; (void)buf; return len; }
