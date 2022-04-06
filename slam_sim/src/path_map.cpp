/***********************************************************************
* Copyright (C) 2019-2022 Keith Godfrey
* Copyright (C) 2022 Gaia
*
* Use of this source code is governed by the MIT
* license that can be found in the LICENSE.txt file
* or at https://opensource.org/licenses/MIT.
***********************************************************************/

/***********************************************************************
* Path-finding logic for occupancy grid. Code here adapted from
* <<TODO provide link or give description>>
***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "occupancy.hpp"


// If node at root+offset belongs as part of path, sets node values
//    (eg, weight and link to parent)
// Add node to stack for future neighbor analysis.
// 'traversal_cost' is the cost to reach this node from the neighbor that
//  added it to the stack.
void occupancy_grid_t::add_node_to_stack(
      /* in     */ const map_node_t& root_node,
      /* in     */ const grid_index_t root_idx,
      /* in     */ const node_offset_t offset,
      /* in     */ const float traversal_cost
      )
{
    // Make sure we're not going off the edge of the map. If so, ignore.
    int32_t new_x = (int32_t) root_node.pos.x + offset.dx;
    int32_t new_y = (int32_t) root_node.pos.y + offset.dy;
    if ((new_x < 0) || (new_x >= m_grid_size.cols) || (new_y < 0) ||
        (new_y >= m_grid_size.rows))
    {
        return;
    }
    /////////////////////////////////////////////////////////////////////
    // Point is in the world. Check it.
    uint32_t new_idx = (uint32_t) (new_x + new_y * m_grid_size.cols);
    map_node_t& child_node = grid->nodes[new_idx];
    if (child_node.flags.state & PATH_NODE_FLAG_IMPASSABLE)
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
    float new_cost = root_node.path_cost + penalty + traversal_wt + jitter;
    if (child_node->flags.state & PATH_NODE_FLAG_PROCESSED) {
        // This node is already-processed. See if this path might provide
        //  it a lower cost.
        if (child_node->path_cost <= new_cost)
        {
            // Nope.
            return;
        }
        // New weight is lower -- allow node to be added to stack again to 
        //    propagate updated weight to neighbors.
    }
    // Add point to stack for future consideration.
    child_node->parent_id = root_idx;
    child_node->weight = new_weight;
    child_node->traversal_cost = penalty;
    child_node->flags.state |= PATH_NODE_FLAG_PROCESSED;
    m_queue.push(new_idx);
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
        return;
    }
    /////////////////////////////////////////////////////////////////////
    // point is in the world -- check it
    uint32_t new_idx = (uint32_t) (new_x + new_y * m_grid_size.cols);
    map_node_t& child_node = m_grid[new_idx];
    if (child_node.flags.state & PATH_NODE_FLAG_IMPASSABLE)
    {
          return;
    }

    // see if there's a way to get to this diagonal node
    // check via vertical and horizontal neighbor
    uint32_t idx_vert =
        (uint32_t) (root_node.pos.x + new_y * m_grid_size.cols);
    uint32_t idx_horiz =
        (uint32_t) (new_x + root_node.pos.y * m_grid_size.cols);
    map_node_t& vert_child_node = m_grid[idx_vert];
    map_node_t& horiz_child_node = m_grid[idx_horiz];
    if ((vert_child_node.flags.state & PATH_NODE_FLAG_IMPASSABLE) &&
        (horiz_child_node.flags.state & PATH_NODE_FLAG_IMPASSABLE))
    {
        // no direct 4-connected path. nothing to do here
        return;
    }
    /////////////////////////////////////////////////////////////////////
    // Point is reachable. Check cost to get there. This will be 
    //  combination of cost from easiest 4-connected path to get 
    //  to diagonal node
    //    and of diagonal node itself
    float cost = -1.0f;
    // get lowest traversal weight of vert and horiz paths and use that as
    //    base for weight to diagonal node
    if ((vert_child_node->flags.state & PATH_NODE_FLAG_IMPASSABLE) == 0) {
        cost = vert_child_node->traversal_cost;
    }
    if ((horiz_child_node->flags.state & PATH_NODE_FLAG_IMPASSABLE) == 0) {
        float h_cost = horiz_child_node->traversal_cost;
        if (cost < 0.0f) {
            // Not accessible by vert node but horiz provides path. Use
            //  horiz penalty.
            cost = h_cost;
        } else if ((h_cost > 0.0f) && (h_cost < cost)) {
            // Horiz node is passable and penalty is less than via vert, so
            //  use Horiz's penalty
            cost = penalty;
        }
    }
    assert(cost >= 0.0);
    // Cost needed to traverse to neighboring node. Scale it up because
    //  distance is farther (it's on a diagonal).
    float traversal_weight = cost * 1.41f;
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
    // Add 4-connected nodes to stack.
    grid_index_t root_idx = m_queue.pop();
    map_node_t& root_node = m_grid[root_idx.idx];
    //
    add_node_to_stack(root_node, root_idx, OFF_E);
    add_node_to_stack(root_node, root_idx, OFF_W);
    add_node_to_stack(root_node, root_idx, OFF_N);
    add_node_to_stack(root_node, root_idx, OFF_S);
    // Add 8-connected nodes to stack. This will only happen if there's
    //  a valid 4-connected path to get there.
    add_node_to_stack_diag(root_node, root_idx, OFF_NE);
    add_node_to_stack_diag(root_node, root_idx, OFF_NW);
    add_node_to_stack_diag(root_node, root_idx, OFF_SE);
    add_node_to_stack_diag(root_node, root_idx, OFF_SW);
}


// Use D* approach to find all routes to destination
// Also builds vector of approximate course to follow for all nodes in path map.
// Present algorithm is very simple and is based on direction to 'grandparent'
//    node, approx. 5 'generations' away.
void occupancy_grid_t::compute_path_costs()
{
    // Update path weight between nodes until stack is empty.
    while (m_queue.size() > 0)
    {
        process_next_stack_node();
    }
    // Now follow gradient in each node to build directional vector 
    //  to reach destination.
    pixel_offset_bitfield_type base_direction, next_direction;
    // Iterate through nodes and build direction vector for each. Vector
    //  is defined by following the path toward the destination (i.e., 
    //  stepping between adjacent nodes) out by several nodes, and measuing
    //  the bearing to that node. If the path turns sharply (e.g., around
    //  an obstacle) then the path is only traced to the turning point.
    for (uint32_t y=0; y<m_grid_size.rows; y++) {
        for (uint32_t x=0; x<m_grid_size.cols; x++) {
            uint32_t idx = x + y * m_grid_size.cols;
            map_node_t& root = path_map->nodes[idx];
            map_node_t& ggp = root;
            // Find direction to ancestor.
            for (uint32_t gen=0; gen<NUM_ANCESTORS_FOR_DIRECTION; gen++) {
                if (ggp.parent_id.val >= 0) 
                {
                    map_node_t& next_ggp = path_map->nodes[ggp.parent_id.idx];
                    if (gen == 0) 
                    {
                        // Get base direction. This returns bitfield 
                        //  indicating direction of next node relative 
                        //  to this one.
                        base_direction = get_offset_mask(next_ggp.pos, ggp.pos);
                    }
                    else
                    {
                        // Make sure offset is consistent w/ base direction
                        //  (ie, w/in 45deg).
                        // Get new direction. This returns bitfield indicating
                        //  direction of next node relative to this one,
                        //  plus adjacent bits. ANDing this to base direction
                        //  will indicate if it's w/in +/-45deg of base.
                        next_direction =
                            get_offset_mask_wide(next_ggp.pos, ggp.pos);
                        if ((next_direction.mask & base_direction.mask) == 0)
                        {
                            // Latest direction is too different from original
                            //    offset. Halt search.
                            break;
                        }
                    }
                    ggp = next_ggp;
                } else {
                    break;
                }
            }
            path_offset_type dir;
            float dx = (float) (ggp->pos.x - root->pos.x);
            float dy = (float) (ggp->pos.y - root->pos.y);
            float theta_degs = R2D * atan2f(dx, -dir.dy);
            if (theta_degs < 0.0)
            {
                theta_degs += 360.0;
            }
            root.direction_degs = theta_degs;
        }
    }
}


// Sets map node to specific weight. This can be a destination node (e.g.,
//  weight=0.0) or a map's boundary conditions (e.g., if embedding a high
//  res map w/in a low res).
void occupancy_grid_t::add_anchor_to_path_stack(
      /* in     */ const grid_index_t idx,
      /* in     */ const float path_weight
      )
{
    assert(idx.idx < m_grid_size.cols * m_grid_size.rows);
    map_node_t& path_node = m_grid[idx.idx];
    path_node.weight = path_weight;
    path_node.parent_id.val = -1;
    m_queue.push(idx);
    path_node->flags.state = PATH_NODE_FLAG_PROCESSED;
}


world_coordinate_t occupancy_grid_t::get_node_location(
    const grid_coordinate_t& pos)
{
    world_coordinate_t world_pos;
    assert(pos.x < m_grid_size.cols);
    assert(pos.y < m_grid_size.rows);
    world_pos.x_meters = m_bottom_left.x_meters + 
        ((float) pos.x + 0.5) * m_node_size_meters;
    world_pos.y_meters = m_bottom_left.y_meters + m_map_size.y_meters -
        ((float) pos.y + 0.5) * m_node_size_meters;
    return world_pos;
}


// Builds map to destination using parent map to set boundary conditions.
//  If destination is outside of map bounds, only parent map is used
//  for path finding.
void occupancy_grid_t::trace_routes(world_coordinate_t destination,
    const occupancy_grid_t& parent_map)
{
    clear();
    // Add anchors. First destination, then boundary conditions.
    grid_index_t idx = get_node_index(destination.x_meters, 
        destination.y_meters;
    add_anchor_to_path_stack(idx, 0.0f);
    // For each boundary point, get distance/weight from parent map at
    //  that location and use that value to set an anchor here.
    for (uint32_t y=0; y<m_grid_size.rows; y++)
    {
        if ((y > 0) && (y < m_grid_size.rows))
        {
            // In this row, anchor first and last columns only
            grid_coordinate_t pos = { .x=x, .y=y };
            world_coordinate_t world_pos = get_node_location(pos);
            // It's assumed that this map is a subset of parent map and
            //  is fully contained in it as that's a design constraint.
            map_node_t& parent_node = parent_map.get_node(world_pos.x_meters, 
                world_pos.y_meters);
            add_anchor_to_path_stack(pos, parent_node.path_cost);
        }
        else
        {
            // This is a top or bottom row. Anchor all nodes in this row.
            for (uint32_t x=0; x<m_grid_size.cols; x++)
            {
                grid_coordinate_t pos = { .x=x, .y=y };
                world_coordinate_t world_pos = get_node_location(pos);
                // It's assumed that this map is a subset of parent map and
                //  is fully contained in it as that's a design constraint.
                map_node_t& parent_node = parent_map.get_node(
                    world_pos.x_meters, world_pos.y_meters);
                add_anchor_to_path_stack(pos, parent_node.path_cost);
            }
        }
    }
    compute_path_costs();
}


void occupancy_grid_t::trace_routes(const world_coordinate_t& destination)
{
    clear();
    grid_index_t idx = get_node_index(destination.x_meters, 
        destination.y_meters0;
    add_anchor_to_path_stack(idx, 0.0f);
    compute_path_costs();
}

