----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database gaia_slam

table graph
(
    uuid string unique,
    vertices references vertex[],
    edges references edge[]
)

table vertex
(
    id uint64 unique,
    type uint8,
    data uint8[],
    pose_x double,
    pose_y double,

    -- Relationships
    graph_uuid string,
    graph references graph
        using vertices
        where vertex.graph_uuid = graph.uuid,

    in_edges references edge[],
    out_edges references edge[]
)

table edge
(
    id uint64 unique,

    -- Relationships
    graph_uuid string,
    graph references graph
        where edge.graph_uuid = graph.uuid,

    dest_id uint64,
    dest references vertex
        using in_edges
        where edge.dest_id = vertex.id,

    src_id uint64,
    src references vertex
        using out_edges
        where edge.src_id = vertex.id
)

------------------------------------------------------------------------
-- Example table. For this example, assume that an entry is made into
--  the 'incoming_data_event' table when a new packet of data arrives. 
table incoming_data_event
(
    id uint64 unique,
    type uint8,
    data uint8[],
    pose_x double,
    pose_y double
)
