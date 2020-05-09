#ifndef __LIBCLI_H__
#define __LIBCLI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdarg.h>
#include <sys/time.h>

#define CLI_OK                  0
#define CLI_ERROR               -1
#define CLI_QUIT                -2
#define CLI_ERROR_ARG           -3

#define MAX_HISTORY             256

#define PRIVILEGE_UNPRIVILEGED  0
#define PRIVILEGE_PRIVILEGED    15
#define MODE_ANY                -1
#define MODE_EXEC               0
#define MODE_CONFIG             1

#define LIBCLI_HAS_ENABLE       1

#define PRINT_PLAIN             0
#define PRINT_FILTERED          0x01
#define PRINT_BUFFERED          0x02

#define CLI_MAX_LINE_LENGTH     4096
#define CLI_MAX_LINE_WORDS      128

#ifndef CLI_BUFFER_SIZE
#define CLI_BUFFER_SIZE         1024
#endif
#ifdef va_copy
#define WL_VA_COPY(ap1, ap2)   va_copy(ap1,ap2)
#else
#define WL_VA_COPY(ap1, ap2)   ((ap1) = (ap2))
#endif

struct cli_def 
{
    int completion_callback;
    struct cli_command *commands;
    int (*auth_callback)(struct cli_def *, const char *, const char *);
    int (*regular_callback)(struct cli_def *cli);
    int (*enable_callback)(struct cli_def *, const char *);
    char *banner;
    struct unp *users;
    char *enable_password;
    char *history[MAX_HISTORY];
    char showprompt;
    char *promptchar;
    char *hostname;
    char *modestring;
    int privilege;
    int mode;
    int state;
    struct cli_filter *filters;
    void (*print_callback)(struct cli_def *cli, const char *string);
    //FILE *client;
    /* internal buffers */
    void *conn;
    void *service;
    char *commandname;  // temporary buffer for cli_command_name() to prevent leak
    char buffer[CLI_BUFFER_SIZE];
    unsigned buf_size;
    int telnet_protocol;
    void *user_context;
    int sockfd;
    ssize_t (*read_callback)(struct cli_def *cli, void *buf, size_t count);
    ssize_t (*write_callback)(struct cli_def *cli, const void *buf, size_t count);
};

struct cli_filter 
{
    int (*filter)(struct cli_def *cli, const char *string, void *data);
    void *data;
    struct cli_filter *next;
};

struct cli_command 
{
    char *command;
    int (*callback)(struct cli_def *, const char *, char **, int);
    unsigned int unique_len;
    char *help;
    int privilege;
    int mode;
    struct cli_command *next;
    struct cli_command *children;
    struct cli_command *parent;
};

struct cli_def *cli_init();
int cli_done(struct cli_def *cli);
struct cli_command *cli_register_command(struct cli_def *cli, struct cli_command *parent, const char *command,
                                         int (*callback)(struct cli_def *, const char *, char **, int), int privilege,
                                         int mode, const char *help);
int cli_unregister_command(struct cli_def *cli, const char *command);
int cli_run_command(struct cli_def *cli, const char *command);
int cli_loop(struct cli_def *cli, int sockfd);
int cli_file(struct cli_def *cli, FILE *fh, int privilege, int mode);
int cli_loop_fd(struct cli_def *cli, int in, int out);
void cli_set_auth_callback(struct cli_def *cli, int (*auth_callback)(struct cli_def *, const char *, const char *));
void cli_set_enable_callback(struct cli_def *cli, int (*enable_callback)(struct cli_def *, const char *));
void cli_allow_user(struct cli_def *cli, const char *username, const char *password);
void cli_allow_enable(struct cli_def *cli, const char *password);
void cli_deny_user(struct cli_def *cli, const char *username);
void cli_set_banner(struct cli_def *cli, const char *banner);
void cli_set_hostname(struct cli_def *cli, const char *hostname);
void cli_set_promptchar(struct cli_def *cli, const char *promptchar);
void cli_set_modestring(struct cli_def *cli, const char *modestring);
int cli_set_privilege(struct cli_def *cli, int privilege);
int cli_set_configmode(struct cli_def *cli, int mode, const char *config_desc);
void cli_reprompt(struct cli_def *cli);
void cli_regular(struct cli_def *cli, int (*callback)(struct cli_def *cli));
void cli_regular_interval(struct cli_def *cli, int seconds);
void cli_print(struct cli_def *cli, const char *format, ...) __attribute__((format (printf, 2, 3)));
void cli_bufprint(struct cli_def *cli, const char *format, ...) __attribute__((format (printf, 2, 3)));
void cli_vabufprint(struct cli_def *cli, const char *format, va_list ap);
void cli_error(struct cli_def *cli, const char *format, ...) __attribute__((format (printf, 2, 3)));
void cli_print_callback(struct cli_def *cli, void (*callback)(struct cli_def *, const char *));
void cli_read_callback(struct cli_def *cli, ssize_t (*callback)(struct cli_def *, void *, size_t));
void cli_write_callback(struct cli_def *cli, ssize_t (*callback)(struct cli_def *, const void *, size_t));
void cli_free_history(struct cli_def *cli);

// Enable or disable telnet protocol negotiation.
// Note that this is enabled by default and must be changed before cli_loop() is run.
void cli_telnet_protocol(struct cli_def *cli, int telnet_protocol);

// Set/get user context
void cli_set_context(struct cli_def *cli, void *context);
void *cli_get_context(struct cli_def *cli);

#ifdef __cplusplus
}
#endif

#endif
