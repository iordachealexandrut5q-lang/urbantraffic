#pragma once

class EdgeOccupant {
public:
	int id = -1;       // vehicle ID
	float progress = 0; // progress along the edge (0.0 to 1.0)
	int lane = 0;      // lane index on the edge (0..3) - 4 lanes total

    EdgeOccupant() = default; 
    EdgeOccupant(int id_, float progress_, int lane_) : id(id_), progress(progress_), lane(lane_) {}
}; 
