#pragma once

class EdgeOccupant {
public:
    int id = -1;       
    float progress = 0; 
    int lane = 0;      

    EdgeOccupant() = default;
    EdgeOccupant(int id_, float progress_, int lane_) : id(id_), progress(progress_), lane(lane_) {}
};
