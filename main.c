#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>


#define LOG_PREFIX "WATCHDOG: "
#define RESTART_TIMEOUT_SEC 5
#define EXIT_TIMEOUT_SEC 2

#define PRINT(stream, fmt, args...) fprintf(stream, LOG_PREFIX fmt "\n", ##args)

#define DEBUG(fmt, args...) PRINT(stdout, fmt, ##args)
#define DEBUG_EXPR(fmt, expr) DEBUG(#expr " = " fmt, expr)

#define ERROR(fmt, args...) PRINT(stderr, "Error: " fmt, ##args)
#define FATAL_ERROR(fmt, args...) {     \
    ERROR(fmt, ##args);                 \
    exit(EXIT_FAILURE);                 \
}
#define FATAL_POSIX_ERROR(desc) FATAL_ERROR("%s: %s", desc, strerror(errno))

#define HANDLE_EINTR(x) ({                                  \
    typeof(x) eintr_wrapper_result;                         \
    do {                                                    \
        eintr_wrapper_result = (x);                         \
    } while(eintr_wrapper_result == -1 && errno == EINTR);  \
    eintr_wrapper_result;                                   \
})

static bool need_exit;
static char **args;
static pid_t child_pid;
static sigset_t old_sigmask;

static int sigfd;
static int timerfd;

static void set_timer(int sec)
{
    DEBUG("Set timer to %d sec", sec);
    struct itimerspec timerspec = {
        .it_value = { .tv_sec = sec }
    };
    if(timerfd_settime(timerfd, 0, &timerspec, NULL) < 0)
        FATAL_POSIX_ERROR("timerfd_settime");
}

static inline bool timer_is_running(void)
{
    struct itimerspec curr_timer;
    if(timerfd_gettime(timerfd, &curr_timer) < 0)
        FATAL_POSIX_ERROR("timerfd_settime");
    return curr_timer.it_value.tv_sec;
}

static void print_args(char **args)
{
    PRINT(stdout, LOG_PREFIX "Run: ");
    char **tmp = args;
    while(*tmp) {
        PRINT(stdout, "%s%s", tmp == args? "" : ", ",  *tmp);
        ++tmp;
    }
    PRINT(stdout, "\n");
}

static void run_prog(void)
{
    print_args(args);
    child_pid = fork();
    if(child_pid > 0) {
        DEBUG("Child PID is %d", child_pid);
        set_timer(RESTART_TIMEOUT_SEC);
    }
    else if(child_pid == 0) {
        if(sigprocmask(SIG_SETMASK, &old_sigmask, NULL) < 0)
            FATAL_POSIX_ERROR("sigprocmask");

        execv(args[0], args);
        FATAL_ERROR("execv %s: %s", args[0], strerror(errno));
    }
    else
        FATAL_POSIX_ERROR("fork");
}

void set_exit(void)
{
    need_exit = true;
    if(!child_pid) {
        DEBUG("Child not running, exit");
        exit(EXIT_SUCCESS);
    }
    else {
        DEBUG("Send SIGTERM to PID %d", child_pid);
        if(kill(child_pid, SIGTERM) < 0)
            FATAL_POSIX_ERROR("kill");
        set_timer(EXIT_TIMEOUT_SEC);
    }
}

void handle_sigchild(void)
{
    DEBUG("Received SIGCHLD");
    int wait_status;
    int rc = HANDLE_EINTR(waitpid(child_pid, &wait_status, WNOHANG));

    if(rc < 0)
        FATAL_POSIX_ERROR("waitpid");

    if(rc > 0) {
        if(WIFEXITED(wait_status)) {
            int exit_status = WEXITSTATUS(wait_status);
            DEBUG("Child exited with status: %d", exit_status);
            if(need_exit) {
                DEBUG("Success exit");
                exit(EXIT_SUCCESS);
            }
            else {
                if(timer_is_running())
                    child_pid = 0;
                else
                    run_prog();
            }
        }
    }
    else {
        FATAL_ERROR("Unexpected SIGCHLD");
    }
}

static int sigfd_create(void)
{
    sigset_t sigmask = {};
    sigaddset(&sigmask, SIGCHLD);
    sigaddset(&sigmask, SIGINT);
    sigaddset(&sigmask, SIGTERM);

    if(sigprocmask(SIG_BLOCK, &sigmask, &old_sigmask) < 0)
        FATAL_POSIX_ERROR("sigprocmask");

    int sigfd = signalfd(-1, &sigmask, SFD_CLOEXEC);
    if(sigfd < 0)
        FATAL_POSIX_ERROR("signalfd");

    return sigfd;
}

static void prepare_watchdog(char **argv)
{
    args = argv;
    timerfd = timerfd_create(CLOCK_REALTIME, TFD_CLOEXEC);
    sigfd = sigfd_create();
}

static void handle_sigfd(struct pollfd *pollfd)
{
    if(pollfd->revents & (POLLERR | POLLNVAL))
        FATAL_ERROR("Received POLLERR or POLLNVAL on sigfd");

    if(pollfd->revents & POLLIN) {
        struct signalfd_siginfo siginfo;
        ssize_t s = HANDLE_EINTR(read(pollfd->fd, &siginfo, sizeof(siginfo)));
        if(s != sizeof(siginfo))
            FATAL_POSIX_ERROR("read sigfd");
        switch(siginfo.ssi_signo) {
            case SIGCHLD:
                handle_sigchild();
                break;

            case SIGINT:
                DEBUG("Received SIGINT");
                set_exit();
                break;

            case SIGTERM:
                DEBUG("Received SIGTERM");
                set_exit();
                break;

            default:
                ERROR("Received an unexpected signal");
        }
    }
    pollfd->revents = 0;
}

static void handle_timerfd(struct pollfd *pollfd)
{
    if(pollfd->revents & (POLLERR | POLLNVAL))
        FATAL_ERROR("Received POLLERR or POLLNVAL on timerfd");

    if(pollfd->revents & POLLIN) {
        if(!need_exit) {
            if(!child_pid)
                run_prog();
            else
                set_timer(0);
        }
        else if(child_pid) {
            DEBUG("Send SIGKILL to PID %d", child_pid);
            if(kill(child_pid, SIGKILL) < 0)
                FATAL_POSIX_ERROR("kill");
            DEBUG("Exit");
            exit(EXIT_SUCCESS);
        }
    }
    pollfd->revents = 0;
}

static void event_loop(void)
{
    assert(sigfd >= 0);
    assert(timerfd >= 0);

    struct pollfd pollfds[] = {
        {
            .fd = sigfd,
            .events = POLLIN | POLLERR,
        },
        {
            .fd = timerfd,
            .events = POLLIN | POLLERR,
        }
    };

    while(true) {
        int rc = poll(pollfds, sizeof(pollfds) / sizeof(*pollfds), 1000);
        if(rc < 0)
            FATAL_POSIX_ERROR("poll");
        if(rc > 0) {
            if(pollfds[0].revents)
                handle_sigfd(pollfds);
            if(pollfds[1].revents)
                handle_timerfd(pollfds + 1);
        }
    }
}

int main(int argc, char **argv)
{
    if(argc < 2)
        FATAL_ERROR("Too few arguments");

    if(access(argv[1], X_OK) < 0)
        FATAL_ERROR("access to %s: %s", argv[1], strerror(errno));

    prepare_watchdog(argv + 1);
    run_prog();
    event_loop();

    return 0;
}
