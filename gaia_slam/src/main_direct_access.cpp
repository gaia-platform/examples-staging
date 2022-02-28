////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <gaia/db/db.hpp>
#include <gaia/direct_access/auto_transaction.hpp>
#include <gaia/logger.hpp>
#include <gaia/rules/rules.hpp>
#include <gaia/system.hpp>

#include "gaia_gaia_slam.h"
#include "graph.hpp"
#include "vertex_types.hpp"

using namespace gaia::direct_access;
using namespace gaia::gaia_slam;
using namespace gaia::gaia_slam::graph;

/**
 * Showcase the usage of the direct_access API to create/read data from the database.
 * This example does not make usage of Rules.
 */
int main()
{
    gaia::system::initialize();

    // This examples wants to focus on direct_access hence the rules are disabled.
    gaia::rules::unsubscribe_rules();

    gaia::db::begin_transaction();

    clear_data();

    graph_t graph = create_graph("graph_1");

    vertex_t v1 = create_vertex(graph, 1, vertex_type::c_lidar_scan);
    vertex_t v2 = create_vertex(graph, 2, vertex_type::c_lidar_scan);
    vertex_t v3 = create_vertex(graph, 3, vertex_type::c_visual_landmark);

    edge_t e1 = create_edge(1, v1, v2);
    edge_t e2 = create_edge(2, v2, v3);

    // Get all graph vertices and edges.

    gaia_log::app().info("=== Printing all vertices from Graph({})  ===", graph.id());

    for (const vertex_t& v : graph.vertices())
    {
        gaia_log::app().info(" Vertex({})", v.id());
    }

    gaia_log::app().info("=== Printing all edges from Graph({}) ===", graph.id());

    for (const edge_t& e : graph.edges())
    {
        gaia_log::app().info(" Edge({}): ({}) --> ({})", e.id(), e.src().id(), e.dest().id());
    }

    gaia_log::app().info("=== Get all vertices with type lidar_scan   ===", graph.id());

    for (const vertex_t& v : vertex_t::list()
                                 .where(vertex_expr::type == vertex_type::c_lidar_scan))
    {
        gaia_log::app().info(" Vertex({})", v.id());
    }

    gaia_log::app().info("=== Get the vertex with id 2 and type lidar_scan ===", graph.id());

    auto vertex_it = vertex_t::list()
                         .where(
                             vertex_expr::id == 2 &&
                             vertex_expr::type == vertex_type::c_lidar_scan);

    if (vertex_it.begin() == vertex_it.end())
    {
        throw std::runtime_error("Cannot find vertex with id 1 and type lidar_scan");
    }

    gaia_log::app().info(" Vertex({})", vertex_it.begin()->id());

    gaia_log::app().info("=== Get all edges with src_id or dest_id equals 1 ===", graph.id());

    auto edge_it = edge_t::list()
                       .where(
                           edge_expr::dest == v1 ||
                           edge_expr::src == v1);

    for (const edge_t& e : edge_it)
    {
        gaia_log::app().info(" Edge({}): ({}) --> ({})", e.id(), e.src().id(), e.dest().id());
    }

    gaia::db::commit_transaction();

    ///
    /// Now you can write your additional logic to use the direct_access API.
    ///

    gaia::system::shutdown();
}
