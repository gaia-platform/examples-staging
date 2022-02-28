----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database gaia_slam

-- SLAM graph.
table graph
(
    id string unique,
    vertices references vertex[],
    edges references edge[]
)

-- A new vertex is created every time a new observation from a sensor is done.
table vertex
(
    id uint64 unique,
    type uint8,
    data uint8[],
    pose_x double,
    pose_y double,

    -- Relationships
    graph_id string,
    graph references graph
        using vertices
        where vertex.graph_id = graph.id,

    in_edges references edge[],
    out_edges references edge[]
)

-- A new edge is created between every new created vertex and the previous one.
table edge
(
    id uint64 unique,

    -- Relationships
    graph_id string,
    graph references graph
        where edge.graph_id = graph.id,

    dest_id uint64,
    dest references vertex
        using in_edges
        where edge.dest_id = vertex.id,

    src_id uint64,
    src references vertex
        using out_edges
        where edge.src_id = vertex.id
)

-- This table represents a generic input from a sensor.
-- i.e. Lidar sensor, image, etc..
-- Note: this is just an example.
table incoming_data_event
(
    id uint64 unique,
    type uint8,
    data uint8[],
    pose_x double,
    pose_y double
)
