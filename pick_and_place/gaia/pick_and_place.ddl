----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database pick_and_place

table state (
    pick bool,
    place bool
)

table pick_action (
    done bool
)

table move_action (
    done bool
)
