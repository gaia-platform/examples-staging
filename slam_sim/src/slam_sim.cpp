#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/random.h>

#include <string>

#include "gaia/system.hpp"

#include "slam_sim.hpp"


namespace slam_sim {

char g_map_json[JSON_BUFFER_LEN] = { 0 };


// Used so clock time starts when simulator starts.
static double g_system_base_time = 0.0;

uint32_t g_quit = 0;


////////////////////////////////////////////////////////////////////////
// Initialization

static void parse_command_line(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    if (argc != 1)
    {
        fprintf(stderr, "Usage: %s\n", argv[0]);
        exit(1);
    }
}

static void init_rand()
{
    // Set random number generator to random initial state.
    union {
        uint8_t buf[4];
        uint32_t val;
    } seed;
    getrandom(seed.buf, 4, GRND_RANDOM | GRND_NONBLOCK);
    srandom(seed.val);
}

static void init_sim()
{
    init_rand();
    //
    strcpy(g_map_json, "{ \"status\": \"unitialized\" }");
    // Set clock base, so 'now()' is 0 when program starts.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_system_base_time = (double) ts.tv_sec + (double) ts.tv_nsec * 1.0e-9;
}

// Initialization
////////////////////////////////////////////////////////////////////////
// Utilities

double now()
{
    // sim time is the amount of actual seconds since base time was
    //  set, multiplied by time compression factor
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double sys_t_sec = (double) ts.tv_sec + (double) ts.tv_nsec * 1.0e-9;
    double sim_sec = sys_t_sec - g_system_base_time;
    return sim_sec;
}

// Sleeps for X seconds of simulated time. Interface is the same
//  as for clock_nanosleep() except that input and output values
//  are doubles and represent sim time.
int sleep_for(double seconds, double* remaining_sec)
{
    struct timespec ts;
    ts.tv_sec = (time_t) seconds;
    ts.tv_nsec = (long) ((seconds - (double) ts.tv_sec) * 1.0e9);
    struct timespec rem;
    int rc = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &rem);
    if (remaining_sec != NULL)
    {
        *remaining_sec = (double) rem.tv_sec + (double) rem.tv_nsec * 1.0e-9;
    }
    return rc;
}

// Utilities
////////////////////////////////////////////////////////////////////////

} // namespace slam_sim


using namespace slam_sim;

int main(int argc, char** argv)
{
    parse_command_line(argc, argv);
    load_default_map();
    signal(SIGINT, slam_sim::thread_shutdown_callback);
    signal(SIGUSR1, slam_sim::thread_shutdown_callback);
    gaia::system::initialize();
    ////////////////////////////////////////////
    // Initialize
    init_sim();
    // Launch communication thread (to receive messages).
    pthread_t tid;
    pthread_create(&tid, NULL, slam_sim::comm_thread_main, NULL);
    ////////////////////////////////////////////
    // Runtime
    while(g_quit == 0)
    {
        // Do nothing. Program actions are managed asynchronously.
        // If a signal is received that'll wake us up and leave the sleep
        //  call.
        sleep_for(10.0, NULL);
    }
    ////////////////////////////////////////////
    // Shutdown
    thread_shutdown_callback(-1);
    pthread_join(tid, NULL);
    gaia::system::shutdown();
    return 0;
}

