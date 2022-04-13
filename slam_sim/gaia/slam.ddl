------------------------------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
------------------------------------------------------------------------

------------------------------------------------------------------------
--
-- Schema for SLAM simulation.
--
-- Schema is split into 3 categories of tables.
--
-- The first includes tables that have individual records in them. For
--  example, the 'ego', the 'destination', and maps that represent the
--  area (used for path finding).
--
-- The second includes tables storing event data. This includes destination
--  requests pushed in externally (e.g., an external request for Alice
--  to go to a particular position) or if Alice senses a collision.
--
-- The third includes the main elements of the SLAM algorithm. This includes
--  graphs, edges, and vertices, and auxiliary tables to support these.
--
------------------------------------------------------------------------

database slam;

-- Table list
-- Single record tables:
--    ego                   Stores state of the bot.
--    area_map              Map of the known world.
--    working_map           Version of world map with recent sensor overlay.
--    destination           Location that bot is moving to.
--    observed_area         Bounds of observed world.
--
-- Event tables:
--    pending_destination   Destination requested externally.
--    collision_event       Sensors detected imminent collision.
--
-- Data tables:
--    graphs                Group of observations to produce pose graph.
--    vertices              Where an observation is made (aka: pose, node).
--      positions               Position associated with observation.
--      movements               Movement to reach this observation.
--      range_data              Sensor range data for an observation.
--    edges                 Connects two observations.
--    error_corrections     Information from graph optimization round.

-- Note that 'vertex' and 'observation' are often used interchangeaby.
-- In future, these should be split. An 'observation' is a view of the
--  environment, and that may or may not become a vertex, while a vertex
--  will always be associated with an observation.

------------------------------------------------------------------------
-- Tables with single records (i.e., exactly one)

-- Record for the bot, storing its state and references to information.
table ego
(
  -- Reference to the active graph. Unlike the references below, this is
  --  a value-linked (implicit) reference that is automatically updated
  --  by updating the 'current_graph_id'.
  current_graph_id uint32,
  current_graph references graphs
      where ego.current_graph_id = graphs.id,

  -- Most recent timestmap is stored in most recent observation/vertex
  --  (referenced below).

  -- Explicitly created references.
  -- Keep most of ego data in different tables so that updates to
  --  one field don't risk conflict and txn rollback.
  -- The references here are for conveninece, as once the 'ego' record
  --  is held there's a direct link to the other tables. Because those
  --  other tables have only one record, they can easily be retrieved
  --  directly (as an alternative to using references).
  latest_observation references latest_observation,

  -- Position oriented.
  destination references destination,
  world references observed_area,

  -- Map oriented.
  low_res_map references area_map,
  working_map references working_map
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


-- Maps would ideally store map content as blobs but max blob size is
--  presently too small. Instead, store pointer to current map and
--  manage concurrent access manually.
table area_map
(
  -- Reference to in-memory blob that stores 2D map.
  -- Blob ID changes on each map update.
  -- Area map must be seeded with a different blob_id than working map.
  blob_id uint32,

  -- Bounding polygon
  -- Uses world coordinates, with increasing X,Y being rightward/upward.
  left_meters float,
  right_meters float,
  top_meters float,
  bottom_meters float,
  -- Grid size
  num_rows uint32,
  num_cols uint32,

  ego references ego,
  working_map references working_map
)


table working_map
(
  -- Reference to in-memory blob that stores 2D map.
  -- Blob ID changes on each map update.
  -- Working map must be seeded with a different blob_id than area map.
  blob_id uint32,

  -- Bounding polygon
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  width_meters float,
  height_meters float,
--  left_meters float,
--  right_meters float,
--  top_meters float,
--  bottom_meters float,
  -- Grid size
  num_rows uint32,
  num_cols uint32,

  -- References
  ego references ego,
  area_map references area_map
)


table destination
(
  -- Uses world coordinates, with increasing X,Y being rightward/upward.
  x_meters float,
  y_meters float,

  -- Keep track of when we left for this destination. If too much time
  --  has elapsed looking for it, abort and go somewhere else.
  departure_time_sec  float,

  -- References
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
  vertex_id uint32,
  vertex references vertices
      where latest_observation.vertex_id = vertices.id,
  ego references ego
)


------------------------------------------------------------------------
-- Event table(s)
-- Pending actions should be put in an event table, as well as unexpected
--  events.

table pending_destination
(
  -- When a destination request is made, it is stored here. When it's
  --  appropriate, this location will become the the new destination.
  -- Uses world coordinates, with increasing X,Y being rightward/upward.
  x_meters float,
  y_meters float
)


table collision_event
(
  description string
)


------------------------------------------------------------------------
-- Data tables (i.e., tables with multiple records)

-- A collection of observations.
table graphs
(
  id uint32 unique,

  vertices references vertices[],
  edges references edges[],

  -- Reverse reference to ego (necessary until one-way references are 
  --  supported)
  ego references ego[]
)


------------------------------------------------
-- Vertex related

-- An observation from a point in the world, including sensor readings
--  from this point and a link to other nearby observations. This is
--  more commonly known as a "node", "pose", or "vertex".
-- Most of observation data in kept different tables so that updates to
--  one field don't risk conflict and txn rollback if others are
--  modified.
table vertices
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
      where graphs.id = vertices.graph_id,

  timestamp_sec double,

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
  vertex references vertices
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
  vertex references vertices
)


table range_data
(
  -- This can be stored in a blob or in dedicated fields.
  num_radials int32,
  bearing_degs float[],
  distance_meters float[],

  -- Constant reference established at record creation time.
  vertex references vertices
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
  src references vertices
      using out_edges where edges.src_id = vertices.id,

  dest_id uint32,
  dest references vertices
      using in_edges where edges.dest_id = vertices.id
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

