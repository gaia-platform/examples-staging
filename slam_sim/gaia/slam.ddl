----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------
database slam;

-- unit suffixes
--    meters
--    degs (degress)
--    sec (seconds)
--    pct (percentage)


------------------------------------------------------------------------
-- Tables with single records (i.e., exactly one)

table ego
(
  current_path_id int32,
  current_path references paths where ego.current_path_id=paths.id,

  -- Explicitly created references.
  -- Keep most of ego data in different tables so that updates to
  --  one field doesn't risk conflict and txn rollback due update
  --  to another.
  position references estimated_position,
  destination references destination,
  low_res_map references area_map,
  high_res_map references local_map
)


-- Maps would ideally store map content as blobs but max blob size is
--  presently too small. Instead, store pointer to current map and
--  manage concurrent access manually.
table area_map
(
  low_res_map_blob_id uint32,
  -- Bounding polygon
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  left_meters float,
  right_meters float,
  top_meters float,
  bottom_meters float,

  -- An on_change rule needs a field to change to fire the rule. Use this
  --  field to guarantee that a change is made so the rule is fired. Note
  --  that a record being "touched" is not sufficient to trigger a change
  --  rule as an actual value change is required.
  change_counter int32,
  ----------------------------
  ego references ego
)


table local_map
(
  high_res_map_blob_id uint32,
  -- Bounding polygon
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  left_meters float,
  right_meters float,
  top_meters float,
  bottom_meters float,

  -- An on_change rule needs a field to change to fire the rule. Use this
  --  field to guarantee that a change is made so the rule is fired. Note
  --  that a record being "touched" is not sufficient to trigger a change
  --  rule as an actual value change is required.
  change_counter int32,
  ----------------------------
  ego references ego,
  working_map references working_map
)


table working_map
(
  working_map_blob_id uint32,

  -- An on_change rule needs a field to change to fire the rule. Use this
  --  field to guarantee that a change is made so the rule is fired. Note
  --  that a record being "touched" is not sufficient to trigger a change
  --  rule as an actual value change is required.
  change_counter int32,
  ----------------------------
  local_map references local_map
)


table destination
(
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  x_meters float,
  y_meters float,

  -- Expected arrival time, in numbers of observations to be taken
  --  on this path. This is useful for aborting trying to reach a
  --  destination.
  -- When set to -1, this means return to a landmark.
  expected_arrival int32,
  ----------------------------
  ego references ego
)


table estimated_position
(
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  x_meters float,
  y_meters float,
  ----------------------------
  ego references ego
)


-- Parameters to estimate tracking errors. This is a running average
--  as estimated over all paths.
table error_correction
(
  drift_correction float,
  forward_correction float,
  correction_weight float
)


------------------------------------------------------------------------
-- Event table(s)
-- Pending actions should be put in an event table, as well as unexpected
--  events.

table pending_destination
(
  -- When a destination request is made, it is stored here. When it's
  --  appriate, this location will become the the new destination.
  -- Uses world coordinates, with increasing X,Y being rightward/upward
  x_meters float,
  y_meters float
)


table collision_event
(
  description string
)


------------------------------------------------------------------------
-- Data tables

-- A sequence of observations between a known start and end point.
--  Start/end points are at distinguishable landmarks.
table paths
(
  id int32 unique,

  ego references ego[],

  state int32,

  -- Observations that are part of this path.
  num_observations uint32,
  start_obs_id int32,
  latest_obs_id int32,
  first_observation references observations
    where paths.start_obs_id = observations.id,
  latest_observation references observations
    where paths.latest_obs_id = observations.id
)


-- An observation from a point in the world, including sensor readings
--  from this point and a link to observations that occurred immediately
--  before and after.
-- This is more typically known as a 'Node' in SLAM. The
--  name difference is because this represents the sensor data from
--  an individual position only. It is not linked together to form
--  a graph of an arbitrary number of nearby pose points, only the
--  previous and next observations. In future iterations, a generic 'node'
--  will be defined, based on their typical definition. It is expected 
--  that the node will be similar to an observation.
table observations
(
  id int32 unique,

  -- DR motion from previous observation
  heading_degs float,
  dist_meters float,

  -- Sensor localization/alignment
  dx_meters float,
  dy_meters float,

  path references paths[] using first_observation,
  path_dup references paths[] using latest_observation,

  ------------------------------
  -- Sensing

  -- Range finder
  num_radials int32,
  distance_meters float[],

  landmark_sightings references landmark_sightings[],

  next_edge references edge,
  prev_edge references edge
)


table edge
(
  id int32, -- record must have a field, not just references

  next references observations using prev_edge,
  prev references observations using next_edge
)


-- A salient feature of the environment that is uniquely identifiable and
--  that can be used to generate a position fix.
table landmarks
(
  landmark_id int32 unique,
  description string unique,

  -- Position of landmark. If this is not supplied externally then it is
  --  generated by Alice. If positions are supplied externally then
  --  their confidence is 1.0. Otherwise, the first landmark encountered
  --  will be assigned a position of 0,0 and a confidence of 1.0.
  --  Subsequent landmarks will have confidences less than 1.0 and
  --  will have approximate positions.
  x_meters float,
  y_meters float,
  confidence float,

  -- Times that this landmark was sighted.
  sightings references landmark_sightings[]
)


-- Locations of observed landmarks.
-- Note that measured distance and range to the landmark will be more
--  accurate the closer Alice is to it.
table landmark_sightings
(
  range_meters float,
  heading_degs float,

  -- Observation corresponding to this sighting.
  observation_id int32,
  observation references observations
    where landmark_sightings.observation_id = observations.id,

  -- Landmark that was sighted
  landmark_id int32,
  landmark references landmarks
    where landmark_sightings.landmark_id = landmarks.landmark_id
)

;

