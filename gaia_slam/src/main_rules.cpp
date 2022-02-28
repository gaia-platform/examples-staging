////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <chrono>

#include <thread>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>
#include <gaia/system.hpp>

#include "gaia_gaia_slam.h"
#include "graph.hpp"
#include "vertex_types.hpp"

using namespace gaia::direct_access;
using namespace gaia::gaia_slam;
using namespace gaia::gaia_slam::graph;

using std::this_thread::sleep_for;

constexpr size_t c_num_data_events = 5;
constexpr uint32_t c_rule_wait_millis = 100;

/**
 * Wait an arbitrary amount of time for rule execution to terminate.
 * Rules are triggered after commit and can take some time to fully execute.
 */
void wait_for_rules()
{
    sleep_for(std::chrono::milliseconds(c_rule_wait_millis));
}

/**
 * Shows how mutating the database triggers the rules in `gaia/gaia_slam.ruleset`.
 */
int main()
{
    gaia::system::initialize();

    // We explicitly handle the transactions with begin_transaction() and commit_transaction()
    // to trigger the rules.
    gaia::db::begin_transaction();
    clear_data();
    gaia::db::commit_transaction();

    gaia_log::app().info("=== Creates a new Graph ===");

    gaia::db::begin_transaction();
    graph_t graph = create_graph("graph_1");
    gaia::db::commit_transaction();

    gaia_log::app().info("=== Inserting {} incoming_data_event of type {} ===", c_num_data_events, vertex_type::c_lidar_scan);

    // Insert some incoming_data_event rows to simulate data arriving from sensors.
    // This should trigger the following rule:
    // - on_insert(incoming_data_event): triggered when an incoming_data_event is inserted. This insert a new vertex.
    gaia::db::begin_transaction();
    for (int i = 0; i < c_num_data_events; i++)
    {
        incoming_data_event_writer data_w;
        data_w.id = i;
        data_w.data = {1, 2, 3, 4, 5};
        data_w.pose_x = i * 2;
        data_w.pose_y = i * 3;
        data_w.type = vertex_type::c_image_keyframe;
        data_w.insert_row();
    }
    gaia::db::commit_transaction();

    wait_for_rules();

    gaia_log::app().info("=== Retrieving vertices/edges with type {} ===", vertex_type::c_image_keyframe);

    // This code verifies that the vertex with type vertex_type::c_image_keyframe has been created.
    gaia::db::begin_transaction();
    for (const vertex_t& v : vertex_t::list()
                                 .where(vertex_expr::type == vertex_type::c_image_keyframe))
    {
        gaia_log::app().info("Vertex(id:{} type:{} pose_x:{} pose_y:{})", v.id(), v.type(), v.pose_x(), v.pose_y());
    }
    gaia::db::commit_transaction();

    ///
    /// Now you can write your additional logic to trigger the other rules,
    /// or write some new rules!
    ///

    gaia::system::shutdown();
}
