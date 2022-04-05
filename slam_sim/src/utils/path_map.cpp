/***********************************************************************
* Copyright (C) 2019-2022 Keith Godfrey
* Copyright (C) 2022 Gaia
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
***********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "occupancy.hpp"


static bool is_not_passable(map_node_t& child_node)
{
    if (child_node.boundary > 0.0)
    {
        // A boundary was observed in this node. If it's not a fluke,
        //  it's not passable.
        assert(child_node.observed > 0.0);
        if (child_node.boundary / child_node.observed > 0.1)
        {
            return;
        }
    }
}


// if node at root+offset belongs as part of path, sets node values
//    (eg, weight and link to parent)
// add pixel to stack for future neighbor analysis
// 'distance' is the cost to reach this node from a neighbor (i.e., 1.0 or
//  1.4, depending if it's NSEW or diagonal neighbor)
void occupancy_grid_t::add_node_to_stack(
      /* in     */ const map_node_t& root_node,
      /* in     */ const grid_index_t root_idx,
      /* in     */ const node_offset_t offset,
      /* in     */ const float traversal_wt
      )
{
   // make sure we're not going off the edge of the map
    int32_t new_x = (int32_t) root_node.pos.x + offset.dx;
    int32_t new_y = (int32_t) root_node.pos.y + offset.dy;
    if ((new_x < 0) || (new_x >= m_grid_size.cols) || (new_y < 0) ||
        (new_y >= m_grid_size.rows))
    {
        return;
    }
   /////////////////////////////////////////////////////////////////////
   // point is in the world -- check it
    uint32_t new_idx = (uint32_t) (new_x + new_y * m_grid_size.cols);
    map_node_t& child_node = grid->nodes[new_idx];
    if (is_not_passable(child_node))
    {
        return;
    }
    // Determine weight for traversing this node. Have lower weight to nodes
    //  infrequently observed/traversed to provide bias to take different
    //  routes.
    float weight = 1.0f;
    weight += child_node.observed * c_path_penalty_per_observation;
    // Path tracing algorithm is deterministic and can follow vert or
    //    horiz path too easily. Add some jitter to allow pulling in
    //    from more accurate direction.
    double jitter = 0.0;
    drand48_r(&path_rand_, &jitter);
    jitter = 0.1 * (jitter - 0.5);

    float new_weight = root_node.weight + penalty + traversal_wt 
        + (float) jitter;
    if (child_node->flags & PATH_NODE_FLAG_PROCESSED) {
        // already-processed node
        // see if this path might provide it a lower weight
        if (child_node->weight <= new_weight)
        {
            // Nope.
            return;
        }
        // new weight is lower -- allow to be added to stack again to propagate
        //    updated weight to neighbors
    }
    // add point to stack for
    child_node->parent_id = root_idx;
    child_node->weight = new_weight;
    child_node->passage_penalty = penalty;
    child_node->flags |= PATH_NODE_FLAG_PROCESSED;
    //
    m_queue.push(new_idx);
end:
   ;
}


// if node at root+offset belongs as part of path, sets node values
//    (eg, weight and link to parent)
// for node that's diagonal, include weight of 4-connected neighbor required
//    to reach diagonal node
// add pixel to stack for future neighbor analysis
void occupancy_grid_t::add_node_to_stack_diag(
      /* in     */ const map_node_t& root_node,
      /* in     */ const grid_index_t root_idx,
      /* in     */ const node_offset_t offset
      )
{
    // make sure we're not going off the edge of the map
    int32_t new_x = (int32_t) root_node.pos.x + offset.dx;
    int32_t new_y = (int32_t) root_node.pos.y + offset.dy;
    if ((new_x < 0) || (new_x >= m_grid_size.cols) 
        || (new_y < 0) || (new_y >= m_grid_size.rows)) 
    {
       goto end;
    }
    /////////////////////////////////////////////////////////////////////
    // point is in the world -- check it
    uint32_t new_idx = (uint32_t) (new_x + new_y * m_grid_size.cols);
    map_node_t& child_node = m_grid[new_idx];
    if (is_not_passable(child_node))
    {
        return;
    }

   // see if there's a way to get to this diagonal node
   // check via vertical and horizontal neighbor
   uint32_t idx_vert =
         (uint32_t) (root_node.pos.x + new_y * m_grid_size.cols);
   uint32_t idx_horiz =
         (uint32_t) (new_x + root_node.pos.y * m_grid_size.cols);
   map_node_t *vert_child_node = &m_grid[idx_vert];
   map_node_t *horiz_child_node = &m_grid[idx_horiz];

   if ((vert_child_node->flags & PATH_NODE_FLAG_NO_ACCESS) &&
         (horiz_child_node->flags & PATH_NODE_FLAG_NO_ACCESS)) {
      // no direct 4-connected path. nothing to do here
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // point is reachable. check weight. this will be combination of
   //    weight from easiest 4-connected path to get to diagonal node
   //    and of diagonal node itself
   float nbr_penalty = -1.0f;
   // get lowest traversal weight of vert and horiz paths and use that as
   //    base for weight to diagonal node
   if ((vert_child_node->flags & PATH_NODE_FLAG_NO_ACCESS) == 0) {
      nbr_penalty = vert_child_node->passage_penalty;
   }
   if ((horiz_child_node->flags & PATH_NODE_FLAG_NO_ACCESS) == 0) {
      float penalty = horiz_child_node->passage_penalty;
      if (nbr_penalty < 0.0f) {
         // not accessible by vert node, but horiz provides path. use
         //    horiz penalty
         nbr_penalty = penalty;
      } else if ((penalty > 0.0f) && (penalty < nbr_penalty)) {
         // horiz node is passable and penalty is less than via vert, so
         //    use horiz's penalty
         nbr_penalty = penalty;
      }
   }
   if (nbr_penalty < 0.0f) {
      // no direct 4-connected path. nothing to do here
      goto end;
   }
   // weight needed to traverse neighboring node
   // diagonal weight should be 1.414, but there appears to be a strong
   //    bias for horiz/vert travel. lower weight to give diagonal more
   //    preference, to balance things out
   float traversal_weight = nbr_penalty + 1.25f;
   add_node_to_stack(root_node, root_idx, offset, traversal_weight);
end:
   ;
}

// pixel neighbors
// 4-connected base
static const node_offset_t OFF_E = { .dx= 1, .dy= 0 };
static const node_offset_t OFF_W = { .dx=-1, .dy= 0 };
static const node_offset_t OFF_N = { .dx= 0, .dy=-1 };
static const node_offset_t OFF_S = { .dx= 0, .dy= 1 };
// 8-connected extras
static const node_offset_t OFF_NE = { .dx= 1, .dy=-1 };
static const node_offset_t OFF_NW = { .dx=-1, .dy=-1 };
static const node_offset_t OFF_SE = { .dx= 1, .dy= 1 };
static const node_offset_t OFF_SW = { .dx=-1, .dy= 1 };


// pop the next node off the stack and processess it
static void occupancy_grod_t::process_next_stack_node()
{
   // add adjacent nodes to stack
   grid_index_t root_idx = m_queue.pop();
   map_node_t& root_node = m_grid[root_idx.idx];
   //
   add_node_to_stack(root_node, root_idx, OFF_E);
   add_node_to_stack(root_node, root_idx, OFF_W);
   add_node_to_stack(root_node, root_idx, OFF_N);
   add_node_to_stack(root_node, root_idx, OFF_S);
   // add 8-connected nodes if there's a valid 2-step 4-conneced path
   //    to get there
   add_node_to_stack_diag(root_node, root_idx, OFF_NE);
   add_node_to_stack_diag(root_node, root_idx, OFF_NW);
   add_node_to_stack_diag(root_node, root_idx, OFF_SE);
   add_node_to_stack_diag(root_node, root_idx, OFF_SW);
}


// build vector of approximate course to follow for all nodes in path map.
// present algorithm is very simple and is based on direction to 'grandparent'
//    node, approx. 5 'generations' away
// TODO present implementation gives very poor angular accuracy and misses
//    turns, potentially aiming it toward land or across shallow water.
//    route calculation should compensate for this but it should be fixed
//    here at the source. build more accurate direction vectors for each
//    path map node
// TODO migrate away from vectors and only calculate course
// also calculates course from vector
// TODO re-evaluate and fix for use in arctic ocean
static void build_course_vectors(
      /* in out */       occupancy_grid_t *path_map,
      /* in     */ const degree_type center_latitude
      )
{
   image_size_type size = path_map->size;
   // build direction vector for each node
   double dy_deg = 1.0 / (double) size.y;
   double y_deg_top = center_latitude.degrees - dy_deg * (double) (size.y / 2);
   pixel_offset_bitfield_type base_direction, next_direction;
   //
   for (uint32_t y=0; y<size.y; y++) {
      double y_deg = y_deg_top + (double) y * dy_deg;
      // apply correction for latitude distortion
      double scale = cos(D2R * y_deg);
      for (uint32_t x=0; x<size.x; x++) {
         uint32_t idx = x + y * size.x;
         map_node_t *root = &path_map->nodes[idx];
         map_node_t *ggp = root;
//printf("%d,%d   scale=%.4f\n", root->pos.x, root->pos.y, scale);
         // find direction to ancestor
         for (uint32_t gen=0; gen<NUM_ANCESTORS_FOR_DIRECTION; gen++) {
            if (ggp->parent_id.val >= 0) {
               map_node_t *next_ggp =
                     &path_map->nodes[ggp->parent_id.idx];
               if (gen == 0) {
                  // get base direction. this returns bitfield indicating
                  //    direction of next node relative to this one
                  base_direction = get_offset_mask(next_ggp->pos, ggp->pos);
               } else {
                  // make sure offset is consistent w/ base direction
                  //    (ie, w/in 45deg)
                  // get new direction. this returns bitfield indicating
                  //    direction of next node relative to this one,
                  //    plus adjacent bits. ANDing this to base direction
                  //    will indicate if it's w/in +/-45deg of base
                  next_direction =
                        get_offset_mask_wide(next_ggp->pos, ggp->pos);
                  if ((next_direction.mask & base_direction.mask) == 0) {
                     // latest direction is too different from original
                     //    offset. halt search
                     break;
                  }
               }
               ggp = next_ggp;
//printf("  ->%d,%d  %.3f\n", ggp->pos.x, ggp->pos.y, ggp->passage_penalty);
            } else {
               break;
            }
         }
         path_offset_type dir;
         dir.dx = (int16_t) (ggp->pos.x - root->pos.x);
         dir.dy = (int16_t) (ggp->pos.y - root->pos.y);
         double theta_deg = R2D * atan2((double) dir.dx * scale, (double) -dir.dy);
//printf("  = %d,%d for %.3f (x-scale= %.3f, wt=%.3f)\n", dir.dx, dir.dy, theta_deg, scale, ggp->weight);
         CVT_DEG_TO_BAM16(theta_deg, root->true_course);
         // don't need to reset active course -- it's set when smoothing
         //    out course
         // active course is reset when smoothing course, but not for all
         //    all nodes (ie, boundary nodes). take care of those cases here
         root->active_course = root->true_course;
//printf("INIT %d   %d,%d [%d]  dir %d,%d  %.2f  (active %.2f)\n", idx, x, y, root->parent_id.val, dir.dx, dir.dy, (double) root->true_course.angle16 * BAM16_TO_DEG, root->active_course.angle16 * BAM16_TO_DEG);
      }
   }
}


// adds point (ie, destination or beacon) to path map, using specified
//    path weight
void occupancy_grid_t::add_destination_to_path_stack(
      /* in     */ const image_coordinate_type pos,
      /* in     */ const float path_weight
      )
{
   // make sure pixel is in map
   if ((pos.x < MAP_LEVEL3_SIZE) || (pos.y < MAP_LEVEL3_SIZE)) {
      uint32_t idx = (uint32_t) (pos.x + pos.y * MAP_LEVEL3_SIZE);
      map_node_t* path_node = m_grid[idx];
      path_node.weight = path_weight;
      path_node.parent_id.val = -1;
      m_queue.push(idx);
      path_node->flags = PATH_NODE_FLAG_PROCESSED;
//fprintf(stderr, "Seeded %d,%d with %.1f\n", pos.x, pos.y, (double) path_weight);
   }
}

// use D* approach to find all routes to destination
// path traced on existing depth map, using beacons and destination as
//    seed locations
void trace_route_simple(
      /*    out */       occupancy_grid_t *path_map,
      /* in     */ const world_coordinate_type vessel_pos
      )
{
//printf("TRACE SIMPLE  %d beacons\n", path_map->num_beacons);
   reset_path_map(path_map);
//   path_map->destination = convert_latlon_to_akn(path_map->center);
   // add beacon and dest weights to map, as able
   path_map->read_idx = 0;
   path_map->write_idx = 0;
   // add destination as primary seed with 0 weight
   // if destination is beyond visible map then it will be filtered and
   //    not actually added stack
   calculate_destination_map_position(path_map);
   add_point_to_path_stack(path_map, path_map->dest_pix, 0.0f);
   // add beacons with seeds of their computed path weights to destination
   for (uint32_t i=0; i<path_map->num_beacons; i++) {
      // TODO don't add beacon if w/in X distance of vessel. that's too
      //    close and likely to result in artifacts
      map_beacon_reference_type *ref = &path_map->beacon_ref[i];
      world_coordinate_type beac_pos = convert_akn_to_world(ref->coords);
      meter_type dist = calc_distance(vessel_pos, beac_pos, __func__);
      if (dist.meters < VESSEL_BEACON_INHIBITION_RING_NM * NM_TO_METERS) {
         // beacon is too close to vessel. inhibit its placement in map
         continue;
      }
      float weight = path_map->beacon_ref[i].path_weight;
      if (weight > 0.0f) {
//printf("  adding beacon %d to stack (wt=%.1f)\n", path_map->beacon_ref[i].index, (double) weight);
         // if using beacon weights directly, variations in paths can
         //    result in a beacon being a local minimum that the path
         //    cannot escape from. to fix this problem, multiply all
         //    beacon weights by a constant (e.g., 2x). this way the
         //    the lowest weight beacon in a map will be the one that
         //    has the strongest attraction, as it should overcome the
         //    weight of all beacons between it and the vessel. this way
         //    we can avoid the local minimum problem without complicated
         //    logic that determines the best beacon to drive toward, and
         //    which are of lower priority (ie, are closer) while covering
         //    for all terrain variations
         add_point_to_path_stack(path_map,
               path_map->beacon_ref[i].pos_in_map, 2.0f * weight);
      }
//else { printf("  beacon %d has wt=%.1f\n", path_map->beacon_ref[i].index, weight); }
   }
   // update path weight between nodes until stack is empty
   uint32_t num_nodes = MAP_LEVEL3_SIZE * MAP_LEVEL3_SIZE;
//uint32_t cnt = 0;
   while (path_map->read_idx < path_map->write_idx) {
//cnt++;
      // because nodes can be added to the stack multiple times it's
      //    possible for stack to grow beyond initial size (which is
      //    number of nodes). if stack overflows, purge used contents
      //    and shrink it
      if (path_map->write_idx >= num_nodes) {
         shrink_path_stack(path_map);
      }
      process_next_stack_node(path_map);
   }
//printf("Processed %d nodes\n", cnt);
   degree_type center_latitude = { .degrees = path_map->center.latitude };
   build_course_vectors(path_map, center_latitude);
}


// generate map based on existing map, with map center being X-nm in
//    front of vessel with vessel following existing path map's route
//    to get to destination
void rebuild_map_by_vessel_offset(
      /* in out */       occupancy_grid_t *path_map,
      /* in     */ const image_coordinate_type vessel_pix,
      /* in     */ const world_coordinate_type vessel_pos
   )
{
printf("Rebuild map by vessel offset\n");
   uint32_t idx = (uint32_t) (vessel_pix.x + vessel_pix.y * path_map->size.x);
   map_node_t *node = &path_map->nodes[idx];
   // if node doesn't have direction information, that's an error, but
   //    tolerate it. can't offset if this node has no path info, so
   //    center map on vessel
   if (node->weight < 0.0f) {
      // can't generate offset if vessel's node has no path info, so just
      //    center map on vessel
fprintf(stderr, "Generating offset map, but vessel pix %d,%d in old map has no path info. Vessel at %.4f,%.4f\n", vessel_pix.x, vessel_pix.y, vessel_pos.lon, vessel_pos.lat);
      log_err(log_, "Generating offset map, but vessel pix %d,%d in old "
            "map has no path info. Vessel at %.4f,%.4f",
            vessel_pix.x, vessel_pix.y, vessel_pos.lon, vessel_pos.lat);
      //
printf("  (no path info)\n");
      load_world_5sec_map(vessel_pos, path_map);
   } else {
      // get direction vessel should be moving and build map based on that
      // direction to head to reach destination
      degree_type course = { .degrees =
            (double) node->true_course.angle16 * BAM16_TO_DEG };
      // calc where map center should be, in that direction
      meter_type dist =
            { .meters = VESSEL_OFFSET_FROM_MAP_CENTER_NM * NM_TO_METERS };
      world_coordinate_type new_center =
            calc_offset_position(vessel_pos, course, dist);
      // load map
      load_world_5sec_map(new_center, path_map);
   }
   load_beacons_into_path_map(path_map);
//write_depth_map(path_map, "c.pnm");
   trace_route_simple(path_map, vessel_pos);
   path_map->vessel_start_pix = get_pix_position_in_map(path_map,
         vessel_pos);
//write_depth_map(path_map, "d.pnm");
}


// initial beacon trace for route plus determining appropriate world
//    map center
// performs several traces. first beacons are traced. then trace is
//    made with vessel at center, then map is moved and trace is made
//    with vessel offset X-nm from center (e.g., 15nm) and direction
//    of travel moving through center
// returns 0 on success and -1 on failure
int trace_route_initial(
      /* in out */       occupancy_grid_t *path_map,
      /* in     */ const world_coordinate_type dest,
      /* in     */ const world_coordinate_type vessel_pos
      )
{
   // TODO either use constants or use size. right now these are
   //    unreasonably intermixed
   assert(path_map->size.x == MAP_LEVEL3_SIZE);
   assert(path_map->size.y == MAP_LEVEL3_SIZE);
   ////////////////
   int rc = -1;
   log_info(log_, "Computing routes from %.5f,%.5f to %.5f,%.5f",
         vessel_pos.x_deg, vessel_pos.y_deg, dest.x_deg, dest.y_deg);
   world_coordinate_type destination = dest;
   if (destination.lon < 0.0) {
      destination.lon += 360.0;
   }
   check_world_coordinate(destination, __func__);
   world_coordinate_type ves_pos = vessel_pos;
   if (ves_pos.lon < 0.0) {
      ves_pos.lon += 360.0;
   }
   check_world_coordinate(ves_pos, __func__);
   path_map->destination = convert_latlon_to_akn(destination);
   // construct beacon path map
   if (trace_beacon_paths(path_map, dest) != 0) {
      // something bad happened. error should have been reported from
      //    w/in function itself
      goto end;
   }
   /////////////////////////////////////////////
   // to calculate map center, first set center as vessel position and
   //    trace path. get direction vector from vessel position and
   //    put map center 15nm ahead of vessel on that vector, and
   //    retrace map
printf("Route initial -- map center\n");
   load_world_5sec_map(ves_pos, path_map);
   load_beacons_into_path_map(path_map);
//write_depth_map(path_map, "a.pnm");
   trace_route_simple(path_map, ves_pos);
//write_depth_map(path_map, "b.pnm");
//write_path_map(path_map, "b.path.pnm");
   image_coordinate_type center =
         { .x = MAP_LEVEL3_SIZE/2, .y = MAP_LEVEL3_SIZE/2 };
   rebuild_map_by_vessel_offset(path_map, center, ves_pos);
   rc = 0;
end:
   return rc;
}

