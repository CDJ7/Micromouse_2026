/*
 * API.c - mms simulator implementation of API.h.
 *
 * Protocol (see the "Mouse API" section of the mms README): we print one
 * command per line on stdout and, for commands that have a reply, read one
 * line from stdin. stdout is reserved for these commands, so anything else
 * (debug logging) must go to stderr. We flush stdout after every command so
 * the simulator sees it immediately.
 */
#include "API.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REPLY_BUF_SIZE 64
#define TEXT_MAX_LEN 10 /* mms limit for cell text */

/* Read one reply line. If the simulator closed the pipe there is nothing left
 * to talk to, so exit instead of spinning forever. */
static void readLine(char *buf, int size) {
    if (fgets(buf, size, stdin) == NULL) {
        fprintf(stderr, "API: simulator closed stdin, exiting\n");
        exit(EXIT_FAILURE);
    }
}

static bool readBool(void) {
    char buf[REPLY_BUF_SIZE];
    readLine(buf, sizeof buf);
    return strncmp(buf, "true", 4) == 0;
}

static int readInt(void) {
    char buf[REPLY_BUF_SIZE];
    readLine(buf, sizeof buf);
    return atoi(buf);
}

/* Read a reply and report whether it was "ack" (anything else, i.e. "crash",
 * counts as failure). */
static bool readAck(void) {
    char buf[REPLY_BUF_SIZE];
    readLine(buf, sizeof buf);
    return strncmp(buf, "ack", 3) == 0;
}

int API_mazeWidth(void) {
    printf("mazeWidth\n");
    fflush(stdout);
    return readInt();
}

int API_mazeHeight(void) {
    printf("mazeHeight\n");
    fflush(stdout);
    return readInt();
}

bool API_wallFront(void) {
    printf("wallFront\n");
    fflush(stdout);
    return readBool();
}

bool API_wallRight(void) {
    printf("wallRight\n");
    fflush(stdout);
    return readBool();
}

bool API_wallLeft(void) {
    printf("wallLeft\n");
    fflush(stdout);
    return readBool();
}

bool API_moveForward(int distance) {
    printf("moveForward %d\n", distance);
    fflush(stdout);
    return readAck(); /* "crash" => false */
}

void API_turnRight(void) {
    printf("turnRight\n");
    fflush(stdout);
    (void)readAck();
}

void API_turnLeft(void) {
    printf("turnLeft\n");
    fflush(stdout);
    (void)readAck();
}

/* The display commands below have no reply, so there is nothing to read. */

void API_setWall(int x, int y, char direction) {
    printf("setWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_clearWall(int x, int y, char direction) {
    printf("clearWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_setColor(int x, int y, char color) {
    printf("setColor %d %d %c\n", x, y, color);
    fflush(stdout);
}

void API_clearColor(int x, int y) {
    printf("clearColor %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllColor(void) {
    printf("clearAllColor\n");
    fflush(stdout);
}

void API_setText(int x, int y, const char *text) {
    /* %.10s truncates for us if the caller passes something too long. */
    printf("setText %d %d %.*s\n", x, y, TEXT_MAX_LEN, text);
    fflush(stdout);
}

void API_clearText(int x, int y) {
    printf("clearText %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllText(void) {
    printf("clearAllText\n");
    fflush(stdout);
}

bool API_wasReset(void) {
    printf("wasReset\n");
    fflush(stdout);
    return readBool();
}

void API_ackReset(void) {
    printf("ackReset\n");
    fflush(stdout);
    (void)readAck();
}
