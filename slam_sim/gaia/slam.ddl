----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------
database slam;

table graph
(
    id int32 unique,
    -- Estimated starting position and heading. Note that this may
    --  be updated as a result of future alignment and optimization.
    --  All obervations that are part of this graph are relative to
    --  these values, w/ the first observation starting at x,y=0,0
    --  and heading_degs=0.
    start_x float,
    start_y float,
    start_heading_degs float,
    --------------------------------------------
    -- Relationships
    --
    num_observations int32,
    first_observation references observations
--    observation_list references observations[]
)

table lidar_data
(
    theta_deg float[],
    distance float[],
    --------------------------------------------
    -- Relationships
    observation references observations
)

table observations
(
    num int32,    -- 0 for first in graph, 1 for 2nd, etc.
    graph_id int32,
    ----------------------------
    -- Sensor-derived motion (e.g., by wheel sensors or IMUs)
    measured_dx float,
    measured_dy float,
    measured_ddegs float,
    ----------------------------
    -- Calculated motion
    est_dx float,
    est_dy float,
    est_heading_ddegs float,
    motion_confidence float,
    ----------------------------
    -- Calculated position. This is relative to the 
    x float,
    y float,
    heading_degs float,
    position_confidence float,
    --------------------------------------------
    -- Relationships
    -- Reference to temporally adjacent observations
    next references observations,
    prev references observations,
    --
    -- Sensor data from this point
    lidar references lidar_data,
    --
    parent references graph
)

