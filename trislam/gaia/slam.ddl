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
--  example, the 'ego' and 'observed area'.
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

-- Single record for storing bot state.
table ego
(
  -- Reference to the active graph. Unlike the references below, this is
  --  a value-linked (implicit) reference that is automatically updated
  --  by updating the 'current_graph_id'.
  current_graph_id uint32,
  current_graph references graphs
      where ego.current_graph_id = graphs.id,

  -- Explicitly created references.
  -- Keep most of ego data in different tables so that updates to
  --  one field don't risk conflict and txn rollback.
  -- The references here are for conveninece, as once the 'ego' record
  --  is held there's a direct link to the other tables. Because those
  --  other tables have only one record, they can easily be retrieved
  --  directly (as an alternative to using references).
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

  ego references ego
)


-- Represents the most recently created observation. 
-- In the context of evaluating the data model for possible transaction
--  conflicts, If multiple observations are not created simultaneously 
--  (which they shouldn't be) then it's not possible to get a transaction
--  collision updating the single record in this table.
-- Represents the most recently created observation.
table latest_observation
(
  observation_id uint32,
  observation references observations
      where latest_observation.observation_id = observations.id,
  ego references ego
)


------------------------------------------------------------------------
-- Data tables (i.e., tables with multiple records)

-- A collection of observations.
table graphs
(
  id uint32 unique,

  observations references observations[],
  edges references edges[],

  -- Reverse reference to ego (necessary until one-way references are 
  --  supported)
  ego references ego[]
)


------------------------------------------------
-- Observation related

-- An observation from a point in the world, including sensor readings
--  from this point and a link to other nearby observations. This is
--  more commonly known as a "node", "pose", or "vertex".
-- Most of observation data in kept different tables so that updates to
--  one field don't risk conflict and txn rollback if others are
--  modified.
table observations
(
  ------------------------------
  -- Constant data. These values are set at creation and don't change.
  --  This is relevant in the sense that, when evaluating for possible
  --  transaction conflicts, these fields/references can be considered
  --  safe.
  id uint32 unique,

  position references positions,
  range_data references range_data,
  motion references movements,

  graph_id uint32,
  graph references graphs
      where graphs.id = observations.graph_id,

  ------------------------------
  -- Dynamic fields. These can be modified asynchronously. Functions
  --  and rules that modify them (i.e., that add edges) should be designed 
  --  to handle a possible transaction rollback unless the logic is
  --  desgined to avoid that.
  in_edges references edges[],
  out_edges references edges[],

  -- Reverse reference to 'latest_observation'. This is not used and
  --  can be removed when one-way references are supported. The []
  --  can be removed when 1:1 VLRs are supported.
  prev_observation references latest_observation[]
)


table positions
(
  -- The latest best guess of position. It may be modified in the future
  --  when we have better error estimates.
  x_meters float,
  y_meters float,
  heading_degs float,

  -- Constant reference established at record creation time.
  observation references observations
)


table movements
(
  -- DR (dead reckoning) motion from previous observation. This is 
  --  a placeholder for storing DR data that could be used during 
  --  graph optimization.
  dx_meters float,
  dy_meters float,
  dheading_degs float,

  -- Constant reference established at record creation time.
  observation references observations
)


table range_data
(
  -- This can be stored in a blob or in dedicated fields.
  num_radials int32,
  bearing_degs float[],
  distance_meters float[],

  -- Constant reference established at record creation time.
  observation references observations
)


------------------------------------------------
-- Other supporting tables.

-- Connects two observations.
table edges
(
  graph_id uint32,
  graph references graphs
      where edges.graph_id = graphs.id,

  -- An edge connects two observations, source and destination. These
  --  have names indicating that they are directed, as they sometimes
  --  will be (observation X -> Y -> Z), but it is noted that many times
  --  they won't be (e.g., if an edge is formed between observation Y
  --  and existing observation M).
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

