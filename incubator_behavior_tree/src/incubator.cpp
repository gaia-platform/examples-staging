////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <cstring>
#include <ctime>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "behavior.hpp"
#include "gaia_behavior.h"
#include "gaia_incubator.h"

using namespace gaia::behavior;

using namespace gaia::common;
using namespace gaia::db;
using namespace gaia::db::triggers;
using namespace gaia::direct_access;
using namespace gaia::incubator;
using namespace gaia::rules;

const char c_sensor_a[] = "Temp A";
const char c_actuator_a[] = "Fan A";

const char c_chicken[] = "chicken";
constexpr float c_chicken_min = 99.0;
constexpr float c_chicken_max = 102.0;

float c_fan_speed_limit = 5000.0;
float c_fan_speed_increment = 500.0;
float c_fan_threshold = 0.7f;

std::atomic<bool> g_in_simulation{false};
std::atomic<int> g_timestamp{0};

gaia_id_t insert_incubator(const char* name, float min_temp, float max_temp)
{
    incubator_writer w;
    w.name = name;
    w.is_on = false;
    w.min_temp = min_temp;
    w.max_temp = max_temp;
    return w.insert_row();
}

void restore_sensor(sensor_t& sensor, float min_temp)
{
    sensor_writer w = sensor.writer();
    w.timestamp = 0;
    w.value = min_temp;
    w.update_row();
}

void restore_actuator(actuator_t& actuator)
{
    actuator_writer w = actuator.writer();
    w.timestamp = 0;
    w.value = 0.0;
    w.update_row();
}

void restore_incubator(incubator_t& incubator, float min_temp, float max_temp)
{
    incubator_writer w = incubator.writer();
    w.is_on = false;
    w.min_temp = min_temp;
    w.max_temp = max_temp;
    w.update_row();

    for (auto& sensor : incubator.sensors())
    {
        restore_sensor(sensor, min_temp);
    }

    for (auto& actuator : incubator.actuators())
    {
        restore_actuator(actuator);
    }
}

void restore_default_values()
{
    for (auto& incubator : incubator_t::list())
    {
        restore_incubator(incubator, c_chicken_min, c_chicken_max);
    }
}

void init_behavioral_tree()
{
    if (node_t::list().size())
    {
        for (auto n : node_t::list())
        {
            auto node_writer = n.writer();
            node_writer.status = 0;
            node_writer.update_row();
        }
        return;
    }

    auto root_node_id = gaia::behavior::node_t::insert_row("root", idle, selector, false, 0, 0);
    auto check_temperature_node_id = gaia::behavior::node_t::insert_row("check_temperature", idle, action, false, check_temperature, 0);
    auto check_fan_node_id = gaia::behavior::node_t::insert_row("check_fan", idle, sequence, false, 0, 1);
    auto set_fan_node_id = gaia::behavior::node_t::insert_row("set_fan", idle, action, false, set_fan_speed, 0);
    auto check_fan_speed_node_id = gaia::behavior::node_t::insert_row("check_fan_adjust", idle, action, false, check_fan_speed, 1);
    auto adjust_fan_node_id = gaia::behavior::node_t::insert_row("adjust_fan", idle, action, false, adjust_fan_speed, 2);

    gaia::behavior::node_t::get(root_node_id).children().connect(check_fan_node_id);
    gaia::behavior::node_t::get(root_node_id).children().connect(check_temperature_node_id);

    gaia::behavior::node_t::get(check_fan_node_id).children().connect(set_fan_node_id);
    gaia::behavior::node_t::get(check_fan_node_id).children().connect(check_fan_speed_node_id);
    gaia::behavior::node_t::get(check_fan_node_id).children().connect(adjust_fan_node_id);
}

void init_storage()
{
    auto_transaction_t tx(auto_transaction_t::no_auto_restart);

    init_behavioral_tree();
    // If we already have inserted an incubator then our storage has already been
    // initialized.  Re-initialize the database to default values.
    if (incubator_t::list().size())
    {
        restore_default_values();
        tx.commit();
        return;
    }

    // Chicken Incubator: 1 sensors, 1 fan
    auto incubator = incubator_t::get(insert_incubator(c_chicken, c_chicken_min, c_chicken_max));
    incubator.sensors().insert(sensor_t::insert_row(c_sensor_a, 0, c_chicken_min));
    incubator.actuators().insert(actuator_t::insert_row(c_actuator_a, 0, 0.0));

    tx.commit();
}

void dump_db()
{
    begin_transaction();
    std::cout << "\n";
    std::cout << std::fixed << std::setprecision(1);
    for (auto i : incubator_t::list())
    {
        std::cout << "-----------------------------------------\n";
        std::printf(
            "%-8s|power: %-3s|min: %5.1lf|max: %5.1lf\n",
            i.name(), i.is_on() ? "ON" : "OFF", i.min_temp(), i.max_temp());
        std::cout << "-----------------------------------------\n";
        for (const auto& s : i.sensors())
        {
            std::printf("\t|%-10s|%10ld|%10.2lf\n", s.name(), s.timestamp(), s.value());
        }
        std::cout << "\t---------------------------------\n";
        for (const auto& a : i.actuators())
        {
            std::printf("\t|%-10s|%10ld|%10.1lf\n", a.name(), a.timestamp(), a.value());
        }
        std::cout << "\t---------------------------------\n";
        for (const auto& n : node_t::list())
        {
            std::printf("\t|%-10s|%d\n", n.name(), n.status());
        }
        std::cout << "\n"
                  << std::endl;
    }
    commit_transaction();
}

float calc_new_temp(float curr_temp, float fan_speed)
{
    const int c_fan_speed_step = 500;
    const float c_temp_cool_step = 0.025;
    const float c_temp_heat_step = 0.1;

    // Simulation assumes we are in a warm environment so if no fans are on then
    // increase the temparature.
    if (fan_speed == 0)
    {
        return curr_temp + c_temp_heat_step;
    }

    // If the fans are <= 500 rpm then cool down slightly.
    int multiplier = static_cast<int>((fan_speed / c_fan_speed_step) - 1);
    if (multiplier <= 0)
    {
        return curr_temp - c_temp_cool_step;
    }

    // Otherwise cool down in proportion to the fan rpm.
    return curr_temp - (static_cast<float>(multiplier) * c_temp_cool_step);
}

float adjust_temperature(float min_temp, float max_temp, float sensor_value, float fan_rpm)
{
    float new_fan_rpm = fan_rpm;
    if (sensor_value >= max_temp)
    {
        new_fan_rpm = std::min(c_fan_speed_limit, fan_rpm + c_fan_speed_increment);
    }
    else if (sensor_value <= min_temp)
    {
        new_fan_rpm = std::max(0.0f, fan_rpm - (2 * c_fan_speed_increment));
    }

    return new_fan_rpm;
}

void simulation()
{
    auto start = std::chrono::steady_clock::now();
    begin_session();

    while (g_in_simulation)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto_transaction_t tx(auto_transaction_t::no_auto_restart);

        float new_temp = 0.0;
        float fan_a = 0.0;

        for (const auto& a : actuator_t::list())
        {
            fan_a = a.value();
        }

        auto current = std::chrono::steady_clock::now();
        g_timestamp = std::chrono::duration_cast<std::chrono::seconds>(current - start).count();
        for (auto s : sensor_t::list())
        {
            sensor_writer w = s.writer();
            new_temp = calc_new_temp(s.value(), fan_a);
            w.value = new_temp;
            w.timestamp = g_timestamp;
            w.update_row();
        }
        // Reset the status of all non running nodes and tick the root node as a new cycle.
        for (auto node : node_t::list())
        {
            auto node_writer = node.writer();
            if (node.status() != running)
            {
                node_writer.status = idle;
            }
            if (strcmp(node.name(), "root") == 0)
            {
                node_writer.tick_flag++;
            }
            node_writer.update_row();
        }

        tx.commit();
    }

    end_session();
}

void usage(const char* command)
{
    std::cout << "Usage: " << command << " [sim|show|help]\n";
    std::cout << " sim: run the incubator simulation.\n";
    std::cout << " show: dump the tables in storage.\n";
    std::cout << " help: print this message.\n";
}

class simulation_t
{
public:
    // Main menu commands.
    static constexpr char c_cmd_begin_sim = 'b';
    static constexpr char c_cmd_end_sim = 'e';
    static constexpr char c_cmd_print_state = 'p';
    static constexpr char c_cmd_quit = 'q';

    // Invalid input.
    const char* c_wrong_input = "Wrong input.";

    void stop()
    {
        if (g_in_simulation)
        {
            g_in_simulation = false;
            m_simulation_thread->join();
            std::cout << "Simulation stopped...\n";
        }
    }

    bool read_input()
    {
        std::getline(std::cin, m_input);
        return !std::cin.eof();
    }

    bool handle_main()
    {
        std::cout << "\n";
        std::cout << "(" << c_cmd_begin_sim << ") | begin simulation\n";
        std::cout << "(" << c_cmd_end_sim << ") | end simulation \n";
        std::cout << "(" << c_cmd_print_state << ") | print current state\n";
        std::cout << "(" << c_cmd_quit << ") | quit\n\n";
        std::cout << "main> ";

        if (!read_input())
        {
            return false;
        }

        if (m_input.size() == 1)
        {
            switch (m_input[0])
            {
            case c_cmd_begin_sim:
                if (!g_in_simulation)
                {
                    g_in_simulation = true;
                    m_simulation_thread = std::make_unique<std::thread>(simulation);
                    std::cout << "Simulation started...\n";
                }
                else
                {
                    std::cout << "Simulation is already running.\n";
                }
                break;
            case c_cmd_end_sim:
                stop();
                break;
            case c_cmd_print_state:
                dump_db();
                break;
            case c_cmd_quit:
                if (g_in_simulation)
                {
                    std::cout << "Stopping simulation...\n";
                    g_in_simulation = false;
                    m_simulation_thread->join();
                    std::cout << "Simulation stopped...\n";
                }
                std::cout << "Exiting..." << std::endl;
                return false;
                break;
            default:
                wrong_input();
                break;
            }
        }
        else
        {
            wrong_input();
        }
        return true;
    }

    int run()
    {
        bool has_input = true;
        while (has_input)
        {
            switch (m_current_menu)
            {
            case menu_t::main:
                has_input = handle_main();
                break;
            default:
                // do nothing
                break;
            }
        }
        stop();
        return EXIT_SUCCESS;
    }

private:
    enum menu_t
    {
        main
    };
    std::string m_input;
    incubator_t m_current_incubator;
    std::unique_ptr<std::thread> m_simulation_thread;
    menu_t m_current_menu = menu_t::main;

    void wrong_input()
    {
        std::cerr << c_wrong_input << std::endl;
    };
};

int main(int argc, const char** argv)
{
    bool is_sim = false;
    std::string server;
    const char* c_arg_sim = "sim";
    const char* c_arg_show = "show";
    const char* c_arg_help = "help";

    if (argc == 2 && strncmp(argv[1], c_arg_sim, strlen(c_arg_sim)) == 0)
    {
        is_sim = true;
    }
    else if (argc == 2 && strncmp(argv[1], c_arg_show, strlen(c_arg_show)) == 0)
    {
        is_sim = false;
    }
    else if (argc == 2 && strncmp(argv[1], c_arg_help, strlen(c_arg_help)) == 0)
    {
        usage(argv[0]);
        return EXIT_SUCCESS;
    }
    else
    {
        if (argc > 1)
        {
            std::cerr << "Wrong arguments." << std::endl;
        }
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (!is_sim)
    {
        gaia::system::initialize();
        gaia::rules::unsubscribe_rules();
        dump_db();
        return EXIT_SUCCESS;
    }

    simulation_t sim;
    gaia::system::initialize();

    std::cout << "-----------------------------------------\n";
    std::cout << "Gaia Incubator\n\n";
    std::cout << "No chickens or puppies were harmed in the\n";
    std::cout << "development or presentation of this demo.\n";
    std::cout << "-----------------------------------------\n";

    init_storage();
    sim.run();
    gaia::system::shutdown();
}
