------------------------------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
------------------------------------------------------------------------

------------------------------------------------------------------------
--
-- Schema for SLAM simulation. This is split into two categories of tables:
--
-- The first includes tables that have individual records in them. For
--  example, the 'ego', the 'position', and maps that represent the
--  area (used for path finding).
--
-- The second includes the main elements of the SLAM algorithm. This includes
--  observations (i.e., 'vertex'), edges and graphs.
--
-- Style note: tables with an individual record in them are named in
--  the singular (e.g., 'ego') while tables that have multiple records
--  in them are named in the plural (e.g., 'observations').
--
------------------------------------------------------------------------

database slam;


------------------------------------------------------------------------
-- Tables with single records (i.e., exactly one)

-- 
table ego
(
  current_graph_id uint32,
  current_graph references graphs
      where ego.current_graph_id = graphs.id,

  -- Explicitly created references.
  -- Keep most of ego data in different tables so that updates to
  --  one field don't risk conflict and txn rollback.
  latest_observation references latest_observation,
  world references observed_area
)


-- Bounds of known world size. When maps are generated, they can use these
--  values for map bounds.
table observed_area
(
  -- Bounding polygon
  -- Uses world coordinates, with increasing X,Y being rightward/upward.
  left_meters float,
  right_meters float,
  top_meters float,
  bottom_meters float,

  ----------------------------
  ego references ego
)


-- Link to most recently created observation. If multiple observations are
--  not created simultaneously (which they shouldn't be) then it's not
--  possible to get a transaction collision updating the single record
--  in this table.
table latest_observation
(
  observation_id uint32,
  observation references observations
      where latest_observation.observation_id = observations.id,
  ego references ego
)


------------------------------------------------------------------------
-- Data tables

-- A collection of observations.
table graphs
(
  id uint32 unique,

  ego references ego[],

  observations references observations[],
  edges references edges[]
)


-- Connects two observations.
table edges
(
  graph_id uint32,
  graph references graphs
      where edges.graph_id = graphs.id,

  -- An edge connects two observations, source and destination. These
  --  don't have to be directed but they can be.
  src_id uint32,
  src references observations
      using out_edges where edges.src_id = observations.id,

  dest_id uint32,
  dest references observations
      using in_edges where edges.dest_id = observations.id
)


-- Created whenever an optimization is performed. Table can store data
--  from that optimization, but more relevantly creating this record also 
--  generates as an event to do post-optimization things, like creating a
--  new map.
table error_corrections
(
  id uint32 unique,
  graph_id uint32

  -- A reference can be made to graphs if necesasry. However, if it's
  --  unlikely to be used, or be used infrequently, might be better
  --  to avoid having reference, to avoid possible transaction conflicts
  --  if modifying graph_id in a transaction that has computationally
  --  intensive actions associated with it.
  -- In the absense of a reference, the correct graph can always be
  --  reached using the following direct access approach:
  --    graphs_t g = *(graphs_t::list()
  --        .where(graphs_t::expr::id == ec.graph_id).begin());
  --  for error_corrections record 'ec'.

  -- Addional data fields here, as necessary.
)

------------------------------------------------------------------------
-- Observation related

-- An observation from a point in the world, including sensor readings
--  from this point and a link to other nearby observations. This is
--  more commonly known as a "node", "pose", or "vertex".
-- Most of observation data in kept different tables so that updates to
--  one field don't risk conflict and txn rollback.
table observations
(
  ----------------------------------------------
  -- Static data. These values are set at creation and don't change.
  id uint32 unique,

  position references positions,
  range_data references range_data,
  motion references movements,
--  ego references ego,

  -- This is set at creation and doesn't change.
  graph_id uint32,
  graph references graphs
      where graphs.id = observations.graph_id,

  ----------------------------------------------
  -- Dynamic fields. These can be modified asynchronously. Functions
  --  and rules that modify them (i.e., that add edges) should be designed 
  --  and expect a possible transaction rollback.
  in_edges references edges[],
  out_edges references edges[],

  prev_observation references latest_observation[]
)


table positions
(
  -- The latest best guess of position. It may be modified in the future
  --  when we have better error estimates.
  pos_x_meters float,
  pos_y_meters float,
  heading_degs float,

  observation references observations
)


table movements
(
  -- DR motion from previous observation. This is a placeholder for storing
  --  dead reckoning data that could be used during graph optimization.
  dx_meters float,
  dy_meters float,
  dheading_degs float,

  observation references observations
)


table range_data
(
  -- This can be stored in a blob or in dedicated fields.
  num_radials int32,
  bearing_degs float[],
  distance_meters float[],

  observation references observations
)

